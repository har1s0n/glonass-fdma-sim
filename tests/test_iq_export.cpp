// tests/test_iq_export.cpp — полнотрактовая проверка экспорта (§7, сценарий Д.10):
// SignalSource → IqSink (CF32) → чтение бинарника ОБРАТНО → побитовая сверка с живыми
// OutputSample. Критерий корректности экспорта — совпадение прочитанного с уже
// провалидированными golden ядра (живой выход step() и есть golden). Спот-пин u[0]=√14
// выведен независимо (не транскрибирован); u[1]≈2,475933500 покрыт round-trip.
#include "glonass/iq_sink.h"
#include "glonass/signal_source.h"
#include "glonass/source_config.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {
// [ТП] те же две точки согласования, что в apps/signal_gen/main.cpp (types.h / nav_message.h).
glonass::PayloadProvider zeroPayload() {
   return [](auto&& /*lineIndex*/) {
             return std::array<std::uint8_t, 76>{};
   }; // [ТП] тип поля
}

// Сценарий Д.10: полная сетка L1, K=-7..+6, Fs=16368000, n0=0, A_k=1, нулевой payload.
glonass::SourceConfig buildD10Config() {
   glonass::SourceConfig cfg;

   cfg.band              = glonass::Band::L1OF; // [ТП] перечислитель
   cfg.sampleRate        = 16368000;
   cfg.referenceFreq     = 1601718750;          // f_центр набора {-7..+6}
   cfg.globalStartSample = 0;

   for (int k = -7; k <= 6; ++k) {
      glonass::LetterConfig lc;
      lc.letter        = k;
      lc.amplitude     = 1.0;
      lc.codePhaseInit = 0.0;
      lc.initialPhase  = 0.0;
      lc.payloadOfLine = zeroPayload();
      cfg.letters.push_back(std::move(lc));
   }
   return cfg;
}

std::uint32_t bitsOf(float v) {
   std::uint32_t b = 0; std::memcpy(&b, &v, sizeof(b)); return b;
}

float floatFromLe(const unsigned char* p) {
   const std::uint32_t b = static_cast<std::uint32_t> (p[0])
                           | (static_cast<std::uint32_t> (p[1]) << 8)
                           | (static_cast<std::uint32_t> (p[2]) << 16)
                           | (static_cast<std::uint32_t> (p[3]) << 24);
   float v = 0.0f; std::memcpy(&v, &b, sizeof(v)); return v;
}
} // namespace

TEST(IqExport, RoundTripBitExactAgainstCore) {
   constexpr int N        = 16;
   const std::string path = "test_iq_export.tmp.cf32";

   glonass::SignalSource source(buildD10Config());

   // Живой выход ядра (эталон round-trip) с одновременной записью.
   std::vector<std::complex<float> > live;

   live.reserve(N);
   {
      glonass::IqSink sink(path);

      for (int i = 0; i < N; ++i) {
         const glonass::OutputSample s = source.step();
         live.push_back(s);
         sink.writeSample(s);
      }
      sink.close();
   }

   // Чтение бинарника обратно (сырой поток CF32 LE).
   std::ifstream in(path, std::ios::binary);
   const std::vector<unsigned char> bytes((std::istreambuf_iterator<char> (in)),
                                          std::istreambuf_iterator<char>());

   // (1) Размер файла == N·8 (без заголовка).
   ASSERT_EQ(bytes.size(), static_cast<std::size_t> (N) * 8);

   // (2) Побитовый round-trip: прочитанное == записанному (I и Q, каждый отсчёт).
   for (int i = 0; i < N; ++i) {
      const float ri = floatFromLe(&bytes[i * 8 + 0]);
      const float qi = floatFromLe(&bytes[i * 8 + 4]);
      EXPECT_EQ(bitsOf(ri), bitsOf(live[i].real())) << "I[" << i << "]";
      EXPECT_EQ(bitsOf(qi), bitsOf(live[i].imag())) << "Q[" << i << "]";
   }

   // (3) Спот-пин n=0: Re u[0]=√14 (независимо выведен), Im u[0]=0 — совпадение с golden ядра.
   //     float32(14/√14) ≡ float32(√14) (проверено), поэтому EXPECT_FLOAT_EQ достаточно.
   EXPECT_FLOAT_EQ(live[0].real(), static_cast<float> (std::sqrt(14.0)));
   EXPECT_FLOAT_EQ(live[0].imag(), 0.0f);
   // u[1]≈2,475933500 (провалидированный golden ядра) — покрыт п.(2): read-back == live[1].

   std::remove(path.c_str());
}
