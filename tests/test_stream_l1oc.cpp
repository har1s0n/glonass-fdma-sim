#include "stream_session.h"

#include "sha256.h"

#include "glonass/types.h"

#include <gtest/gtest.h>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

// Режим Б микросервиса L1OC — потоковая выдача отсчётов (GET /v1/stream)
//
// Контрольные значения пересчитаны независимо: docs/raschet_l1oc/gate_l1oc_stream.py,
// вывод docs/raschet_l1oc/gate_l1oc_stream_out.txt. Хеш конфигурации К4 совпал с контрольным
// значением Д_L1OC.10 (получен собственным счётом до сверки).

namespace {
using glonass_params::ParamError;
using glonass_params::RejectKind;
using glonass_service::parseStreamRequest;
using glonass_service::quantizationScaleCs16;
using glonass_service::SampleFormat;
using glonass_service::StreamRequest;
using glonass_service::StreamSession;

constexpr std::int64_t carrier    = glonass::carrierFreqL1OC;
constexpr std::int64_t sampleRate = 20000000; // Fs = 20,0 МГц (§ 0.1 поз.35)

std::vector<int> range(int first, int last) {
   std::vector<int> values;

   for (int value = first; value <= last; ++value) {
      values.push_back(value);
   }
   return values;
}

// Конфигурация прогона: J = {first…last}, A_j = 1, φ_{0,j} = 0, f₀ = f_L1OC
StreamRequest configuration(std::int64_t startSample, std::int64_t sampleCount,
                            int first, int last, SampleFormat format,
                            std::int64_t blockSamples = glonass_service::defaultBlockSamples) {
   StreamRequest request;

   request.sampleRate    = sampleRate;
   request.referenceFreq = carrier;
   request.startSample   = startSample;
   request.satellites    = range(first, last);
   request.amplitudes    = std::vector<double> (request.satellites.size(), 1.0);
   request.initialPhases = std::vector<double> (request.satellites.size(), 0.0);
   request.sampleCount   = sampleCount;
   request.blockSamples  = blockSamples;
   request.format        = format;
   return request;
}

// Полный поток одной сессии → SHA-256. Блоки подаются в хеш по мере выдачи: запись целиком
// не накапливается, как и в самой точке.
std::string streamSha256(const StreamRequest& request) {
   StreamSession session(request);
   testutil::Sha256 hash;

   for (std::span<const unsigned char> block = session.nextBlock(); !block.empty();
        block = session.nextBlock()) {
      hash.update(block.data(), block.size());
   }
   return hash.hexDigest();
}

// Размеры выданных блоков до исчерпания потока
std::vector<std::size_t> blockSizes(const StreamRequest& request) {
   StreamSession session(request);
   std::vector<std::size_t> sizes;

   for (std::span<const unsigned char> block = session.nextBlock(); !block.empty();
        block = session.nextBlock()) {
      sizes.push_back(block.size());
   }
   return sizes;
}

// Байты одного отсчёта потока по индексу выдачи r — для сверки не только по хешу
std::vector<unsigned char> sampleBytes(const StreamRequest& request, std::int64_t index) {
   StreamRequest single = request;

   single.blockSamples = 1;
   StreamSession session(single);
   std::span<const unsigned char> block;

   for (std::int64_t position = 0; position <= index; ++position) {
      block = session.nextBlock();
   }
   return std::vector<unsigned char> (block.begin(), block.end());
}
} // namespace

// ───────────────────────────── разбор параметров ─────────────────────────────

TEST(StreamRequestL1OC, Test1_DefaultsFollowContract) {
   const httplib::Request request;
   const StreamRequest    parsed = parseStreamRequest(request);

   EXPECT_EQ(parsed.sampleRate,    20000000);
   EXPECT_EQ(parsed.referenceFreq, carrier);
   EXPECT_EQ(parsed.startSample,   0);
   EXPECT_EQ(parsed.satellites.size(), 24U); // J = {1,…,24}
   EXPECT_EQ(parsed.blockSamples, 65536);    // контракт § 5.2
   EXPECT_EQ(parsed.format, SampleFormat::cs16);

   // Без n и seconds поток бесконечен (§ 5.2): умолчание модуля запуска здесь не применяется
   EXPECT_EQ(parsed.sampleCount, 0);
}

TEST(StreamRequestL1OC, Test2_SampleCountTakesPrecedenceOverSeconds) {
   httplib::Request request;

   request.params.emplace("n",       "1000");
   request.params.emplace("seconds", "0.008"); // дало бы 160000
   EXPECT_EQ(parseStreamRequest(request).sampleCount, 1000);
}

// n = round(seconds·Fs), половина — ОТ нуля (§ 0.1 поз.20): при Fs = 20 МГц произведение ровно
// 0,5 достигается на seconds = 2,5·10⁻⁸; округление к чётному дало бы 0.
TEST(StreamRequestL1OC, Test3_SecondsRoundsHalfAwayFromZero) {
   httplib::Request half;

   half.params.emplace("seconds", "2.5e-8");
   EXPECT_EQ(parseStreamRequest(half).sampleCount, 1);

   httplib::Request window;

   window.params.emplace("seconds", "0.008"); // окно 8 мс — период замыкания кодовой фазы
   EXPECT_EQ(parseStreamRequest(window).sampleCount, 160000);
}

TEST(StreamRequestL1OC, Test4_NonPositiveDurationRejected) {
   for (const char* value : { "0", "-1" }) {
      httplib::Request request;

      request.params.emplace("n", value);

      try {
         parseStreamRequest(request);
         ADD_FAILURE() << "ожидался отказ для n = " << value;
      } catch (const ParamError& error) {
         EXPECT_EQ(error.kind(), RejectKind::badValue);
         EXPECT_EQ(error.field(), "n");
      }
   }
   httplib::Request tooShort;

   tooShort.params.emplace("seconds", "1e-9"); // 0,02 отсчёта → 0
   EXPECT_THROW(parseStreamRequest(tooShort), ParamError);
}

TEST(StreamRequestL1OC, Test5_BlockSamplesBounded) {
   for (const char* value : { "1", "65536", "1048576" }) {
      httplib::Request request;

      request.params.emplace("blockSamples", value);
      EXPECT_NO_THROW(parseStreamRequest(request)) << "blockSamples = " << value;
   }

   for (const char* value : { "0", "-1", "1048577" }) {
      httplib::Request request;

      request.params.emplace("blockSamples", value);

      try {
         parseStreamRequest(request);
         ADD_FAILURE() << "ожидался отказ для blockSamples = " << value;
      } catch (const ParamError& error) {
         EXPECT_EQ(error.kind(), RejectKind::badValue);
         EXPECT_EQ(error.field(), "blockSamples");
      }
   }
}

TEST(StreamRequestL1OC, Test6_FormatLimitedToTwoValues) {
   httplib::Request cf32;

   cf32.params.emplace("format", "cf32");
   EXPECT_EQ(parseStreamRequest(cf32).format, SampleFormat::cf32);

   httplib::Request cs16;

   cs16.params.emplace("format", "cs16");
   EXPECT_EQ(parseStreamRequest(cs16).format, SampleFormat::cs16);

   httplib::Request unknown;

   unknown.params.emplace("format", "int8");

   try {
      parseStreamRequest(unknown);
      ADD_FAILURE() << "ожидался отказ для неизвестного формата";
   } catch (const ParamError& error) {
      EXPECT_EQ(error.kind(), RejectKind::badValue);
      EXPECT_EQ(error.field(), "format");
   }
}

// В отличие от режима А, точка выполняет прогон: нарушение В.2 отклоняется ДО начала выдачи
// (разряд unrealizable → 422), а не выводится полем ответа.
TEST(StreamRequestL1OC, Test7_NonRepresentableRejectedBeforeOutput) {
   httplib::Request request;

   request.params.emplace("fs", "4091999"); // |Δf| + B_model > Fs/2 при f₀ = f_L1OC

   try {
      parseStreamRequest(request);
      ADD_FAILURE() << "ожидался отказ по условию представимости В.2";
   } catch (const ParamError& error) {
      EXPECT_EQ(error.kind(), RejectKind::unrealizable);
      EXPECT_EQ(error.field(), "fs");
   }
   httplib::Request boundary;

   boundary.params.emplace("fs", "4092000"); // равенство принимается (условие нестрогое)
   EXPECT_NO_THROW(parseStreamRequest(boundary));
}

// ───────────────────────────── разбиение на блоки ─────────────────────────────

// Разд. 1 пересчёта: блоков = ⌈n/b⌉, последний = n − b·(блоков−1)
TEST(StreamSessionL1OC, Test8_BlockPartition) {
   const std::size_t cf32Block = 65536U * 8U;

   EXPECT_EQ(blockSizes(configuration(0, 200000, 1, 1, SampleFormat::cf32)),
             (std::vector<std::size_t>{ cf32Block, cf32Block, cf32Block, 3392U * 8U }));

   // Короткий последний блок в один отсчёт
   EXPECT_EQ(blockSizes(configuration(0, 65537, 1, 1, SampleFormat::cf32)),
             (std::vector<std::size_t>{ cf32Block, 8U }));

   // n < blockSamples: блок не дополняется до blockSamples
   EXPECT_EQ(blockSizes(configuration(0, 1, 1, 1, SampleFormat::cf32)),
             (std::vector<std::size_t>{ 8U }));

   // CS16 — 4 байта на отсчёт (контракт § 6)
   EXPECT_EQ(blockSizes(configuration(0, 200000, 1, 1, SampleFormat::cs16)),
             (std::vector<std::size_t>{ 65536U * 4U, 65536U * 4U, 65536U * 4U, 3392U * 4U }));
}

TEST(StreamSessionL1OC, Test9_UnboundedStreamKeepsEmittingFullBlocks) {
   StreamRequest request = configuration(0, 0, 1, 1, SampleFormat::cf32, 4096);
   StreamSession session(request);

   EXPECT_EQ(session.sampleCount(), 0); // предела нет

   for (int block = 1; block <= 5; ++block) {
      EXPECT_EQ(session.nextBlock().size(), 4096U * 8U);
      EXPECT_EQ(session.samplesEmitted(), 4096LL * block);
   }
}

// Разд. 5 пересчёта: blockSamples на содержание потока не влияет
TEST(StreamSessionL1OC, Test10_BlockSizeDoesNotAffectContent) {
   const StreamRequest base = configuration(40000, 4096, 1, 1, SampleFormat::cf32);
   const std::string   expected =
      "80184c4f0080d5deb1af3987fdea750f5870809c143492ab853fd58c74db1812";

   for (const std::int64_t blockSamples : { 1, 4096, 65536 }) {
      StreamRequest request = base;

      request.blockSamples = blockSamples;
      EXPECT_EQ(streamSha256(request), expected) << "blockSamples = " << blockSamples;
   }
}

// ───────────────────────── побитовый эталон CF32 ─────────────────────────

// Разд. 3 пересчёта: поток обязан совпадать с файлом .cf32 модуля запуска побайтно
TEST(StreamSessionL1OC, Test11_Cf32MatchesLaunchModuleRecord) {
   EXPECT_EQ(streamSha256(configuration(0, 200000, 1, 1, SampleFormat::cf32)),
             "5fa0acf004a8d77fbd6a3368831b5b99bc337add6ad4d0adb8ab99fb34c6d3ee") << "К1";
   EXPECT_EQ(streamSha256(configuration(0, 65537, 1, 3, SampleFormat::cf32)),
             "30abc401ec7fa78e6250ad9fcfe72738b918a844687cdc193063a17f3d3a50fe") << "К2";
   EXPECT_EQ(streamSha256(configuration(40000, 4096, 1, 1, SampleFormat::cf32)),
             "80184c4f0080d5deb1af3987fdea750f5870809c143492ab853fd58c74db1812") << "К3";

   // К4 — сценарий Д_L1OC.10: 24 НКА, окно 8 мс
   EXPECT_EQ(streamSha256(configuration(0, 160000, 1, 24, SampleFormat::cf32)),
             "64478a9168144182e50b3f031bc0ad54c6b649907fa5d0a24446f8d0f7716353") << "К4";
}

TEST(StreamSessionL1OC, Test12_Cf32BlockHashes) {
   StreamSession session(configuration(0, 200000, 1, 1, SampleFormat::cf32));
   const std::vector<std::string> expected = {
      "5ee4ea4cd830ba79dc7d96b0193d459d4fa080ef5e6a8714f9f307c87a71f245",
      "9f9469b2fa157b37903a32520776e996323f5cc1ea2ed397d76b050cf210a750",
      "ac8deaafd6dd3df9f25e52745659475dab6a63fc2afba9e821d4d9d8e481c668",
      "6c828ac05add563dd2246cef9311060d3740ce7b86db08267c318cf74a04d6a1"
   };
   std::vector<std::string> actual;

   for (std::span<const unsigned char> block = session.nextBlock(); !block.empty();
        block = session.nextBlock()) {
      testutil::Sha256 hash;

      hash.update(block.data(), block.size());
      actual.push_back(hash.hexDigest());
   }
   EXPECT_EQ(actual, expected);
}

// Q[n] ≡ +0,0 при φ_{0,j} = 0 (§ 2_L1OC.5, Д_L1OC.11). Накопление блока Д_L1OC начинается с
// нуля, поэтому (+0,0) + (−0,0) = +0,0: байты Q равны 00 00 00 00, а не 00 00 00 80.
TEST(StreamSessionL1OC, Test13_ExactSampleBytes) {
   const std::vector<unsigned char> first =
      sampleBytes(configuration(0, 200000, 1, 1, SampleFormat::cf32), 0);

   EXPECT_EQ(first, (std::vector<unsigned char>{ 0x00, 0x00, 0x80, 0xBF,   // I = −1
                                                 0x00, 0x00, 0x00, 0x00 })); // Q = +0,0

   // |J| = 3: I = −η = −0,57735025882720947 после приведения к float32
   EXPECT_EQ(sampleBytes(configuration(0, 65537, 1, 3, SampleFormat::cf32), 0),
             (std::vector<unsigned char>{ 0x3A, 0xCD, 0x13, 0xBF,
                                          0x00, 0x00, 0x00, 0x00 }));

   // Привязка n₀ = 40000: первый отсчёт потока — отсчёт n = 40000
   EXPECT_EQ(sampleBytes(configuration(40000, 4096, 1, 1, SampleFormat::cf32), 0),
             (std::vector<unsigned char>{ 0x00, 0x00, 0x80, 0x3F,
                                          0x00, 0x00, 0x00, 0x00 }));

   // К4, r = 196: Σg = 24 — аналитическая граница η·Σ A_j = √24 достигается фактически
   EXPECT_EQ(sampleBytes(configuration(0, 160000, 1, 24, SampleFormat::cf32), 196),
             (std::vector<unsigned char>{ 0x71, 0xC4, 0x9C, 0x40,
                                          0x00, 0x00, 0x00, 0x00 }));
}

// ───────────────────────────── формат CS16 ─────────────────────────────

// Разд. 4.1 пересчёта: k = 32767/(η·Σ A_j); при A_j = 1 граница равна √|J|
TEST(StreamSessionL1OC, Test14_QuantizationScaleFromAnalyticBound) {
   EXPECT_DOUBLE_EQ(quantizationScaleCs16(std::vector<double> (1,  1.0)), 32767.0);
   EXPECT_DOUBLE_EQ(quantizationScaleCs16(std::vector<double> (3,  1.0)), 18918.036270536464);
   EXPECT_DOUBLE_EQ(quantizationScaleCs16(std::vector<double> (8,  1.0)), 11584.883949569803);
   EXPECT_DOUBLE_EQ(quantizationScaleCs16(std::vector<double> (24, 1.0)), 6688.535866814699);

   // Масштаб фиксируется на прогон и не зависит от выданных отсчётов (п. 10.1)
   StreamSession session(configuration(0, 200000, 1, 24, SampleFormat::cs16));
   const double before = session.quantizationScale();

   session.nextBlock();
   EXPECT_DOUBLE_EQ(session.quantizationScale(), before);
   EXPECT_DOUBLE_EQ(before, 6688.535866814699);
}

// Разд. 4.4 пересчёта: поток CS16 при k = k_анал. Совпадает с выходом
// glonass_iq_convert --scale k побитно; умолчание конвертера (пик записи) в потоке недоступно.
TEST(StreamSessionL1OC, Test15_Cs16MatchesAnalyticScale) {
   EXPECT_EQ(streamSha256(configuration(0, 200000, 1, 1, SampleFormat::cs16)),
             "55a49c121fa1c3f887aef26e58e80508c761fdc64c2c753c49fb4724c6bd733e") << "К1";
   EXPECT_EQ(streamSha256(configuration(0, 65537, 1, 3, SampleFormat::cs16)),
             "d3d6d1ec4dfad7c739f5937277542e67fc916e5e418dce2cbbafdff940a83b4b") << "К2";
   EXPECT_EQ(streamSha256(configuration(40000, 4096, 1, 1, SampleFormat::cs16)),
             "631b410b98f6c8298aa0040866ab444456734003ecdaa9fbdf8eda97e4875188") << "К3";
   EXPECT_EQ(streamSha256(configuration(0, 160000, 1, 24, SampleFormat::cs16)),
             "6505180fc177195c7dcdf33fd77546fa128a87793bfa6836389310723b2bb783") << "К4";
}

// Ограничение ±32767 при аналитическом масштабе не срабатывает по построению, знак Q не
// инвертируется, Q = 0 при φ_{0,j} = 0.
TEST(StreamSessionL1OC, Test16_Cs16ReachesFullScaleWithoutClipping) {
   StreamSession session(configuration(0, 200000, 1, 1, SampleFormat::cs16));
   int  maxMagnitude = 0;
   bool quadratureIsZero = true;

   for (std::span<const unsigned char> block = session.nextBlock(); !block.empty();
        block = session.nextBlock()) {
      for (std::size_t offset = 0; offset < block.size(); offset += 4) {
         const auto valueAt = [&block](std::size_t position) {
                                 return static_cast<std::int16_t> (
                                    static_cast<std::uint16_t> (block[position])
                                    | (static_cast<std::uint16_t> (block[position + 1]) << 8));
                              };
         const int magnitude = std::abs(static_cast<int> (valueAt(offset)));

         if (magnitude > maxMagnitude) {
            maxMagnitude = magnitude;
         }

         if (valueAt(offset + 2) != 0) {
            quadratureIsZero = false;
         }
      }
   }
   EXPECT_EQ(maxMagnitude, 32767); // |J| = 1: граница достигается, усечения нет
   EXPECT_TRUE(quadratureIsZero);
}
