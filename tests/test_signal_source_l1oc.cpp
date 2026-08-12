// tests/test_signal_source_l1oc.cpp — источник суммарного сигнала тракта L1OC (§ 2_L1OC).
// Сборка А_L1OC+Б_L1OC+В+Г_L1OC+Д_L1OC в двухфазном поотсчётном шаге; сверка с контрольными
// значениями Ч3 Д_L1OC.10 (η, контрольные отсчёты, отпечатки потоков I[n] и CF32), независимо
// пересчитанными скриптом docs/raschet_l1oc/gate_l1oc_step.py.
#include "glonass/iq_sink.h"
#include "glonass/signal_source_l1oc.h"
#include "glonass/source_config_l1oc.h"
#include "glonass/types.h"
#include "sha256.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace glonass;
using testutil::Sha256;

namespace {
constexpr std::int64_t kFsL1OC   = 20000000; // Fs = 20,0 МГц (§ 0.1 поз.35)
constexpr int          kWindow8m = 160000;   // окно 8 мс — период замыкания кодовой фазы (Д_L1OC.10)
constexpr double       kPi       = 3.14159265358979323846;

// Режим контрольных примеров Д_L1OC.10: ЦИ нулевая, строка нормального типа (Б_L1OC.11).
PayloadProviderL1OC zeroProvider() {
   return [](std::int64_t) {
             return LineContentL1OC{};
   };
}

// J = {1,…,size} — младшие системные номера, j = 0 резервный (поз.28); A_j, φ_{0,j} — общие.
SourceConfigL1OC makeConfig(int         size,
                            double      amplitude = 1.0,
                            double      phi0      = 0.0,
                            SampleIndex n0        = 0) {
   SourceConfigL1OC cfg;

   cfg.sampleRate        = kFsL1OC;
   cfg.referenceFreq     = carrierFreqL1OC; // f₀ = f_L1OC ⇒ Δf_j = 0 (поз.26)
   cfg.globalStartSample = n0;

   for (int j = 1; j <= size; ++j) {
      SatelliteConfigL1OC sc;

      sc.satellite         = j;
      sc.amplitude         = amplitude;
      sc.initialPhase      = phi0;
      sc.payloadOfLineL1OC = zeroProvider();
      cfg.satellites.push_back(sc);
   }
   return cfg;
}

std::uint32_t bitsOf(float value) {
   std::uint32_t bits = 0;

   std::memcpy(&bits, &value, sizeof(bits));
   return bits;
}

// Байты float32 младшим байтом вперёд — формат выхода CF32 LE (Д_L1OC.11; iq_sink.h).
void updateFloatLe(Sha256& hash, float value) {
   const std::uint32_t bits     = bitsOf(value);
   const std::uint8_t  bytes[4] = {
      static_cast<std::uint8_t> (bits         & 0xFFu),
      static_cast<std::uint8_t> ((bits >> 8)  & 0xFFu),
      static_cast<std::uint8_t> ((bits >> 16) & 0xFFu),
      static_cast<std::uint8_t> ((bits >> 24) & 0xFFu),
   };

   hash.update(bytes, sizeof(bytes));
}

float floatFromLe(const unsigned char* p) {
   const std::uint32_t bits = static_cast<std::uint32_t> (p[0])
                              | (static_cast<std::uint32_t> (p[1]) << 8)
                              | (static_cast<std::uint32_t> (p[2]) << 16)
                              | (static_cast<std::uint32_t> (p[3]) << 24);
   float value = 0.0f;

   std::memcpy(&value, &bits, sizeof(value));
   return value;
}
} // namespace

// --- Тест 1: состав J, коэффициент нормировки η и индекс отсчёта (Д_L1OC.1), (Д_L1OC.4) ---
// Таблица η контрольного примера Д_L1OC.10; при A_j = 1 ⇒ η = 1/√|J|. n = n₀ + r (§ 2_L1OC.5).
TEST(SignalSourceL1OCConfig, Test1_ActiveSetEtaAndSampleIndex) {
   struct Reference {
      int    size;
      double eta;
      double tolerance;
   };
   const Reference references[5] = {
      {  1, 1.000000000,       0.0   }, // |J| = 1 ⇒ η = 1 точно (golden, Д_L1OC.4)
      {  2, 0.707106781186547, 1e-15 },
      {  8, 0.353553390593274, 1e-15 },
      { 24, 0.204124145231932, 1e-15 },
      { 63, 0.125988158,       1e-9  }, // Д_L1OC.10 приводит 9 знаков
   };

   for (const Reference& r : references) {
      SignalSourceL1OC source(makeConfig(r.size));

      EXPECT_EQ(source.satelliteCount(), static_cast<std::size_t> (r.size));
      EXPECT_NEAR(source.normalizationFactor(), r.eta, r.tolerance) << "|J|=" << r.size;
   }

   // неравные амплитуды: A = {1, 2} ⇒ η = 1/√5 (Д_L1OC.10)
   SourceConfigL1OC unequal = makeConfig(2);

   unequal.satellites[1].amplitude = 2.0;
   EXPECT_NEAR(SignalSourceL1OC(unequal).normalizationFactor(), 0.447213595499958, 1e-15);

   // индекс отсчёта: n = n₀ + r — индекс СЛЕДУЮЩЕГО выдаваемого отсчёта
   SignalSourceL1OC fromZero(makeConfig(1));

   EXPECT_EQ(fromZero.sampleIndex(), SampleIndex{ 0 });

   for (int r = 0; r < 5; ++r) {
      fromZero.step();
   }
   EXPECT_EQ(fromZero.sampleIndex(), SampleIndex{ 5 });

   SignalSourceL1OC fromOffset(makeConfig(1, 1.0, 0.0, 1000));

   EXPECT_EQ(fromOffset.sampleIndex(), SampleIndex{ 1000 });
   fromOffset.step();
   fromOffset.step();
   EXPECT_EQ(fromOffset.sampleIndex(), SampleIndex{ 1002 });
}

// --- Тест 2: контрольные отсчёты I[n] = η·Σ_j g_j[n] (Д_L1OC.10) ---
// Окно 8 мс, ЦИ нулевая, φ_{c0,j} = 0, φ_{0,j} = 0, n₀ = 0, Fs = 20,0 МГц.
TEST(SignalSourceL1OCVector, Test2_ControlSamples) {
   const int points[6] = { 0, 1, 20, 40, 79, 159999 };

   struct Reference {
      int    size;
      double outputs[6]; // I[n]
   };
   const Reference references[3] = {
      {  1, { -1.0, -1.0, 1.0, 1.0, 1.0,      -1.0 } }, // Σ g_j = −1,−1,+1,+1,+1,−1
      {  2, {  0.0, 0.0,  0.0, 0.0, 1.414214, 0.0  } }, // Σ g_j = 0,0,0,0,+2,0
      { 24, {  0.0, 0.0,  0.0, 0.0, 0.0,      0.0  } }, // Σ g_j = 0 во всех точках
   };

   for (const Reference& r : references) {
      SignalSourceL1OC source(makeConfig(r.size));
      std::size_t      next = 0;

      for (int n = 0; n <= points[5]; ++n) {
         const OutputSample sample = source.step();

         if ((next < 6) && (n == points[next])) {
            EXPECT_NEAR(sample.real(), r.outputs[next], 1e-6) << "|J|=" << r.size << " n=" << n;
            EXPECT_FLOAT_EQ(sample.imag(), 0.0f) << "|J|=" << r.size << " n=" << n; // e_j ≡ 1 ⇒ Q ≡ 0
            ++next;
         }
      }
      ASSERT_EQ(next, 6u) << "|J|=" << r.size;
   }
}

// --- Тест 3: отпечатки потоков на окне 8 мс (Д_L1OC.10) ---
// Сборка целиком, включая блок В (Δθ_j = 0 ⇒ e_j ≡ 1): SHA-256 потока I[n] (float32 LE) и
// потока CF32 LE с чередованием I,Q — совпадение с Ч3 подтверждает и порядок фаз шага.
TEST(SignalSourceL1OCVector, Test3_Sha256Streams) {
   struct Reference {
      int         size;
      const char* sha256Inphase;
      const char* sha256Interleaved; // nullptr — в Д_L1OC.10 отпечаток не приведён
   };
   const Reference references[3] = {
      {  1, "66c216877694c6a6741c78cf4fdb43292452a783fc52e2a72cfd86e2d4be0863",
         "2accb8d0b396c97492c556ec79e1d82183911cf4aee745d7185c5390dcf2f61b" },
      {  2, "a1c105eff8469b5406c39159e9e090608fd33cc80682113974ee342c62145fd2", nullptr },
      { 24, "4b63b35848946626447799e56879f1e5fb6010961801d91861444eb56029e181",
         "64478a9168144182e50b3f031bc0ad54c6b649907fa5d0a24446f8d0f7716353" },
   };

   for (const Reference& r : references) {
      SignalSourceL1OC source(makeConfig(r.size));
      Sha256 inphaseHash;
      Sha256 interleavedHash;

      for (int n = 0; n < kWindow8m; ++n) {
         const OutputSample sample = source.step();

         updateFloatLe(inphaseHash,     sample.real());
         updateFloatLe(interleavedHash, sample.real());
         updateFloatLe(interleavedHash, sample.imag());
      }
      EXPECT_EQ(inphaseHash.hexDigest(), r.sha256Inphase) << "|J|=" << r.size;

      if (r.sha256Interleaved != nullptr) {
         EXPECT_EQ(interleavedHash.hexDigest(), r.sha256Interleaved) << "|J|=" << r.size;
      }
   }
}

// --- Тест 4: порядок накопления суммы задаётся возрастанием j, а не порядком конфигурации ---
// Источник сортирует J по возрастанию (Д_L1OC.8, § 2_L1OC.3): конфигурация, поданная по
// убыванию j, даёт побитово тот же поток. Начальные фазы разнесены (φ_{0,j} = 2π·idx/|J|) —
// вклады комплексные, сложение неассоциативно и прямой обход расходится с обратным (Д_L1OC.10);
// при A_j = 1 и φ_{0,j} = 0 сумма целая и от порядка не зависит, такой набор сортировку не
// проверяет.
TEST(SignalSourceL1OCOrder, Test4_ConfigOrderIndependence) {
   constexpr int size = 24;

   SourceConfigL1OC ascending = makeConfig(size);

   for (int j = 1; j <= size; ++j) {
      ascending.satellites[static_cast<std::size_t> (j - 1)].initialPhase =
         2.0 * kPi * (j - 1) / size;
   }
   SourceConfigL1OC descending;

   descending.sampleRate        = ascending.sampleRate;
   descending.referenceFreq     = ascending.referenceFreq;
   descending.globalStartSample = ascending.globalStartSample;

   for (int j = size; j >= 1; --j) {
      descending.satellites.push_back(ascending.satellites[static_cast<std::size_t> (j - 1)]);
   }
   ASSERT_EQ(descending.satellites.front().satellite, size);

   SignalSourceL1OC ascendingSource(ascending);
   SignalSourceL1OC descendingSource(descending);

   for (int n = 0; n < 2000; ++n) {
      const OutputSample expected = ascendingSource.step();
      const OutputSample actual   = descendingSource.step();

      ASSERT_EQ(bitsOf(actual.real()), bitsOf(expected.real())) << "n=" << n;
      ASSERT_EQ(bitsOf(actual.imag()), bitsOf(expected.imag())) << "n=" << n;
   }
}

// --- Тест 5: квадратурное положение φ_{0,j} = π/2 (§ 0.1 поз.46), (В.4) ---
// Θ_j[0] = 2^(B−2) ⇒ e_j = −0,0 + i·1,0 точно; вклад g_j·e_j переносится в квадратуру:
// I[n] ≡ 0, а поток Q[n] побитово совпадает с потоком I[n] при φ_{0,j} = 0 (Д_L1OC.10).
TEST(SignalSourceL1OCCarrier, Test5_QuadratureInitialPhase) {
   SignalSourceL1OC source(makeConfig(1, 1.0, kPi / 2.0));
   Sha256 quadratureHash;

   for (int n = 0; n < kWindow8m; ++n) {
      const OutputSample sample = source.step();

      ASSERT_EQ(std::abs(sample.real()), 0.0f) << "n=" << n; // I[n] = ±0,0
      ASSERT_EQ(std::abs(sample.imag()), 1.0f) << "n=" << n; // Q[n] = ±1,0 (η = 1, A_j = 1)
      updateFloatLe(quadratureHash, sample.imag());
   }
   EXPECT_EQ(quadratureHash.hexDigest(),
             "66c216877694c6a6741c78cf4fdb43292452a783fc52e2a72cfd86e2d4be0863");
}

// --- Тест 6: привязка к шкале T_ГЛ (вход n₀, § 0.1 поз.25) ---
// Источник, запущенный от n₀, побитово совпадает с источником от 0, прокрученным на n₀ шагов:
// init всех блоков от n₀ (А_L1OC.5, Б_L1OC.9, В.4) эквивалентен прогону от начала шкалы.
TEST(SignalSourceL1OCStart, Test6_GlobalStartSampleEquivalence) {
   constexpr SampleIndex n0 = 160000; // 8 мс: замыкание кодовой фазы, две границы символа ОК1

   SignalSourceL1OC fromZero(makeConfig(2));
   SignalSourceL1OC fromOffset(makeConfig(2, 1.0, 0.0, n0));

   for (SampleIndex n = 0; n < n0; ++n) {
      fromZero.step();
   }
   ASSERT_EQ(fromZero.sampleIndex(), fromOffset.sampleIndex());

   for (int n = 0; n < 2000; ++n) {
      const OutputSample expected = fromZero.step();
      const OutputSample actual   = fromOffset.step();

      ASSERT_EQ(bitsOf(actual.real()), bitsOf(expected.real())) << "n=" << n;
      ASSERT_EQ(bitsOf(actual.imag()), bitsOf(expected.imag())) << "n=" << n;
   }
}

// --- Тест 7: амплитуды A_j и η раздельны (§ 0.1 поз.24), (Д_L1OC.8) ---
// A_j входит во вклад u_j[n] в блоке Г_L1OC, в блоке Д_L1OC применяется только η; двойной
// нормировки нет: |J| = 1, A = 2 ⇒ η = 0,5 ⇒ |I[n]| = 1 точно.
TEST(SignalSourceL1OCAmplitude, Test7_AmplitudeAndEtaAreSeparate) {
   SignalSourceL1OC scaled(makeConfig(1, 2.0));

   EXPECT_DOUBLE_EQ(scaled.normalizationFactor(), 0.5); // η = 1/√4

   for (int n = 0; n < 1024; ++n) {
      const OutputSample sample = scaled.step();

      ASSERT_EQ(std::abs(sample.real()), 1.0f) << "n=" << n; // 2·0,5 = 1 — приведение точно
      ASSERT_EQ(sample.imag(),           0.0f) << "n=" << n;
   }

   // A = {1, 2} ⇒ η = 1/√5; сумма вкладов ±1 ± 2 ⇒ |I[n]| ∈ {1/√5, 3/√5}, оба значения встречаются
   SourceConfigL1OC unequal = makeConfig(2);

   unequal.satellites[1].amplitude = 2.0;
   SignalSourceL1OC source(unequal);
   const float      etaFloat = static_cast<float> (1.0 / std::sqrt(5.0));
   int    smallCount         = 0;
   int    largeCount         = 0;

   for (int n = 0; n < 5000; ++n) {
      const OutputSample sample = source.step();
      const float        level  = std::abs(sample.real());

      if (std::abs(level - etaFloat) < 1e-6f) {
         ++smallCount;
      } else if (std::abs(level - 3.0f * etaFloat) < 1e-6f) {
         ++largeCount;
      } else {
         FAIL() << "n=" << n << " I=" << sample.real(); // иных уровней быть не может
      }
   }
   EXPECT_GT(smallCount, 0);
   EXPECT_GT(largeCount, 0);
}

// --- Тест 8: экспорт потока тракта L1OC (Д_L1OC.11; iq_sink.h) ---
// SignalSourceL1OC → IqSink (CF32 LE) → чтение бинарника обратно → побитовая сверка с живым
// выходом ядра; размер файла = N·8 байт (сырой поток, заголовка нет).
TEST(SignalSourceL1OCExport, Test8_IqSinkRoundTrip) {
   constexpr int     count = 16;
   const std::string path  = "test_signal_source_l1oc.tmp.cf32";

   SignalSourceL1OC source(makeConfig(24));
   std::vector<OutputSample> live;

   live.reserve(count);
   {
      IqSink sink(path);

      for (int n = 0; n < count; ++n) {
         const OutputSample sample = source.step();

         live.push_back(sample);
         sink.writeSample(sample);
      }
      sink.close();
      EXPECT_EQ(sink.samplesWritten(), static_cast<std::size_t> (count));
   }

   std::ifstream in(path, std::ios::binary);
   const std::vector<unsigned char> bytes((std::istreambuf_iterator<char> (in)),
                                          std::istreambuf_iterator<char> ());

   ASSERT_EQ(bytes.size(), static_cast<std::size_t> (count) * 8);

   for (int n = 0; n < count; ++n) {
      const float readI = floatFromLe(&bytes[static_cast<std::size_t> (n) * 8 + 0]);
      const float readQ = floatFromLe(&bytes[static_cast<std::size_t> (n) * 8 + 4]);

      EXPECT_EQ(bitsOf(readI), bitsOf(live[static_cast<std::size_t> (n)].real())) << "I[" << n << "]";
      EXPECT_EQ(bitsOf(readQ), bitsOf(live[static_cast<std::size_t> (n)].imag())) << "Q[" << n << "]";
   }
   std::remove(path.c_str());
}
