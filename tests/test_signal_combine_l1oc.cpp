#include "glonass/signal_combine.h"
#include "glonass/types.h"
#include "sha256.h"
#include "source_l1oc.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <set>
#include <span>
#include <vector>

using namespace glonass;
using namespace testutil;

namespace {
class ActiveSetL1OC {
public:

   explicit ActiveSetL1OC(int size)
      : amplitudes_(static_cast<std::size_t> (size), 1.0), // A_j = 1 (§ 0.1 позиция 24)
      sourceSamples_(static_cast<std::size_t> (size)) {
      sources_.reserve(static_cast<std::size_t> (size));

      for (int j = 1; j <= size; ++j) {
         sources_.emplace_back(j);
      }
   }

   // фаза 1: u_j[n] = A_j·g_j[n]·e_j[n] (Г_L1OC.5), по возрастанию j (Д_L1OC.8)
   std::span<const std::complex<double> > collect(std::span<const std::complex<double> > phasors = {}) {
      for (std::size_t i = 0; i < sources_.size(); ++i) {
         const std::complex<double> carrier = phasors.empty() ? kUnitPhasor : phasors[i];

         sourceSamples_[i] = sources_[i].sourceSample(carrier, amplitudes_[i]);
      }
      return sourceSamples_;
   }

   // фаза 2: обновление состояний блоков А_L1OC и Б_L1OC к n+1 (§ 2_L1OC.3)
   void advance() {
      for (SourceL1OC& source : sources_) {
         source.step();
      }
   }

   std::span<const double> amplitudes() const {
      return amplitudes_;
   }

   std::size_t size() const {
      return sources_.size();
   }

private:

   std::vector<SourceL1OC> sources_;                  // по возрастанию j
   std::vector<double> amplitudes_;                   // A_j
   std::vector<std::complex<double> > sourceSamples_; // u_j[n], буфер переиспользуется
};

double realSum(std::span<const std::complex<double> > sourceSamples) {
   double sum = 0.0;

   for (const std::complex<double>& sourceSample : sourceSamples) {
      sum += sourceSample.real();
   }
   return sum;
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
} // namespace

// --- Тест 1: коэффициент нормировки η = 1/√(Σ_{j∈J} A_j²) (Д_L1OC.1), (Д_L1OC.4) ---
// Контрольный пример Д_L1OC.10, таблица η; при A_j = 1 ⇒ η = 1/√|J|.
TEST(SignalCombineL1OCNormalization, Test1_EtaByActiveSetSize) {
   struct Reference {
      int    size;
      double eta;
      double tolerance;
   };
   const Reference references[6] = {
      {  2, 0.707106781186547, 1e-15                                                                                     },
      {  8, 0.353553390593274, 1e-15                                                                                     },
      { 12, 0.288675134594813, 1e-15                                                                                     },
      { 24, 0.204124145231932, 1e-15                                                                                     },
      { 63, 0.125988158,       1e-9                                                                                      }, // Д_L1OC.10
                                                                                                                            // приводит 9
                                                                                                                            // знаков
      { 64, 0.125,             0.0                                                                                       }, // 1/√64 = 0,125
                                                                                                                            // —
                                                                                                                            // представимо
                                                                                                                            // точно
   };

   for (const Reference& r : references) {
      const std::vector<double> amplitudes(static_cast<std::size_t> (r.size), 1.0);

      EXPECT_NEAR(normalizationFactor(amplitudes), r.eta, r.tolerance) << "|J|=" << r.size;
   }

   // |J| = 1 ⇒ η = 1 точно: golden-режим, огибающая ±1 (Д_L1OC.4)
   const std::vector<double> single(1, 1.0);

   EXPECT_DOUBLE_EQ(normalizationFactor(single), 1.0);

   // неравные амплитуды: A = {1, 2} ⇒ η = 1/√5 (Д_L1OC.10)
   const std::vector<double> unequal = { 1.0, 2.0 };

   EXPECT_NEAR(normalizationFactor(unequal), 0.447213595499958, 1e-15);
}

// --- Тест 2: нормировка по средней мощности η²·Σ A_j² = 1 (Д_L1OC.8) ---
// η² = 1/(Σ_j A_j²) задаёт номинальную мощность P_nom = E{|u[n]|²} = 1; проверка тождества
// на равных и неравных амплитудах.
TEST(SignalCombineL1OCNormalization, Test2_AveragePowerIdentity) {
   const std::vector<std::vector<double> > amplitudeSets = {
      std::vector<double> (1,  1.0),
      std::vector<double> (2,  1.0),
      std::vector<double> (24, 1.0),
      std::vector<double> (63, 1.0),
      { 1.0,                   2.0 },
      { 0.5,                   1.5,                  2.0, 0.25 },
   };

   for (const std::vector<double>& amplitudes : amplitudeSets) {
      double sumSquares = 0.0;

      for (double amplitude : amplitudes) {
         sumSquares += amplitude * amplitude;
      }
      const double eta = normalizationFactor(amplitudes);

      EXPECT_NEAR(eta * eta * sumSquares, 1.0, 1e-15) << "|J|=" << amplitudes.size();
   }
}

// --- Тест 3: домножение на η ПОСЛЕ суммирования; двойной нормировки нет (Д_L1OC.8) ---
// A_j входит во вклад u_j[n] в блоке Г_L1OC (Г_L1OC.5), в блоке Д_L1OC применяется только η
// (§ 0.1 позиция 24). Суммирование раздельное по координатам (Д_L1OC.3).
TEST(SignalCombineL1OCScaling, Test3_ScalingAfterSummation) {
   const std::vector<double> amplitudes = { 1.0, 2.0 };
   const double eta                     = normalizationFactor(amplitudes); // 1/√5

   // вещественные вклады: u_1 = A_1·(+1), u_2 = A_2·(−1) ⇒ Σ = −1
   const std::vector<std::complex<double> > sourceSamples = {
      { +1.0, 0.0 }, { -2.0, 0.0 }
   };
   const OutputSample sample = combine(sourceSamples, eta);

   EXPECT_FLOAT_EQ(sample.real(), static_cast<float> (-eta));
   EXPECT_FLOAT_EQ(sample.imag(), 0.0f);

   // комплексные вклады: суммирование раздельно по I и Q, затем общий множитель η
   const std::vector<std::complex<double> > complexSamples = {
      { 0.6, 0.8 }, { -1.2, -1.6 }
   };
   const OutputSample complexSample = combine(complexSamples, eta);

   EXPECT_FLOAT_EQ(complexSample.real(), static_cast<float> (eta * -0.6));
   EXPECT_FLOAT_EQ(complexSample.imag(), static_cast<float> (eta * -0.8));

   // амплитуда не применяется повторно: |u| одного источника с A = 2 равен 2·η
   const std::vector<std::complex<double> > singleSample = { { 0.0, 2.0 } };

   EXPECT_FLOAT_EQ(combine(singleSample, eta).imag(), static_cast<float> (2.0 * eta));
}

// --- Тест 4: приведение double→float32 в конце, единичное округление на координату (Д_L1OC.9) ---
// η ведётся в double; предварительное округление η до float32 дало бы иной результат.
// Различающий случай: |J| = 5 (η = 1/√5), Σ_j g_j = 3 ⇒ расхождение конвенций в 1 ULP.
TEST(SignalCombineL1OCNumeric, Test4_SingleRoundingToFloat32) {
   const std::vector<double> amplitudes(5, 1.0);
   const double eta                                       = normalizationFactor(amplitudes);
   const std::vector<std::complex<double> > sourceSamples = {
      { +1.0, 0.0 }, { +1.0, 0.0 }, { +1.0, 0.0 }, { +1.0, 0.0 }, { -1.0, 0.0 }
   };

   ASSERT_DOUBLE_EQ(realSum(sourceSamples), 3.0);

   const OutputSample sample = combine(sourceSamples, eta);

   EXPECT_EQ(bitsOf(sample.real()), bitsOf(static_cast<float> (eta * 3.0)));
   EXPECT_NE(bitsOf(sample.real()), bitsOf(static_cast<float> (eta) * 3.0f));
}

// --- Тест 5: контрольные отсчёты Σ_j g_j[n] и I[n] = η·Σ (Д_L1OC.10) ---
// Окно 8 мс, ЦИ нулевая
TEST(SignalCombineL1OCVector, Test5_ControlSamples) {
   const int points[6] = { 0, 1, 20, 40, 79, 159999 };

   struct Reference {
      int    size;
      double sums[6];    // Σ_j g_j[n]
      double outputs[6]; // I[n] = η·Σ
   };
   const Reference references[3] = {
      {  1, { -1, -1, 1,  1,  1,  -1    }, { -1.0, -1.0, 1.0,  1.0,  1.0,       -1.0                                      } },
      {  2, {  0, 0,  0,  0,  2,  0     }, {  0.0, 0.0,  0.0,  0.0,  1.414214,  0.0                                       } },
      { 24, {  0, 0,  0,  0,  0,  0     }, {  0.0, 0.0,  0.0,  0.0,  0.0,       0.0                                       } },
   };

   for (const Reference& r : references) {
      ActiveSetL1OC activeSet(r.size);
      const double  eta  = normalizationFactor(activeSet.amplitudes());
      std::size_t   next = 0;

      for (int n = 0; n <= points[5]; ++n) {
         const std::span<const std::complex<double> > sourceSamples = activeSet.collect();

         if ((next < 6) && (n == points[next])) {
            const OutputSample sample = combine(sourceSamples, eta);

            EXPECT_DOUBLE_EQ(realSum(sourceSamples), r.sums[next])
               << "|J|=" << r.size << " n=" << n;
            EXPECT_NEAR(sample.real(), r.outputs[next], 1e-6) << "|J|=" << r.size << " n=" << n;
            EXPECT_FLOAT_EQ(sample.imag(), 0.0f) << "|J|=" << r.size << " n=" << n;
            ++next;
         }
         activeSet.advance();
      }
      ASSERT_EQ(next, 6u) << "|J|=" << r.size;
   }
}

// --- Тест 6: отпечатки потоков на окне 8 мс (Д_L1OC.10) ---
// SHA-256 потока I[n] (float32 LE, 160 000 отсчётов) и потока CF32 LE с чередованием I,Q.
TEST(SignalCombineL1OCVector, Test6_Sha256Streams) {
   struct Reference {
      int         size;
      const char* sha256Inphase;
      const char* sha256Interleaved; // nullptr — в Д_L1OC.10 отпечаток не приведён
   };
   const Reference references[3] = {
      {  1, "66c216877694c6a6741c78cf4fdb43292452a783fc52e2a72cfd86e2d4be0863",
         "2accb8d0b396c97492c556ec79e1d82183911cf4aee745d7185c5390dcf2f61b" },
      {  2, "a1c105eff8469b5406c39159e9e090608fd33cc80682113974ee342c62145fd2",nullptr        },
      { 24, "4b63b35848946626447799e56879f1e5fb6010961801d91861444eb56029e181",
        "64478a9168144182e50b3f031bc0ad54c6b649907fa5d0a24446f8d0f7716353" },
   };

   for (const Reference& r : references) {
      ActiveSetL1OC activeSet(r.size);
      const double  eta = normalizationFactor(activeSet.amplitudes());
      Sha256 inphaseHash;
      Sha256 interleavedHash;

      for (int n = 0; n < samplesPer8ms; ++n) {
         const OutputSample sample = combine(activeSet.collect(), eta);

         updateFloatLe(inphaseHash,     sample.real());
         updateFloatLe(interleavedHash, sample.real());
         updateFloatLe(interleavedHash, sample.imag());
         activeSet.advance();
      }
      EXPECT_EQ(inphaseHash.hexDigest(), r.sha256Inphase) << "|J|=" << r.size;

      if (r.sha256Interleaved != nullptr) {
         EXPECT_EQ(interleavedHash.hexDigest(), r.sha256Interleaved) << "|J|=" << r.size;
      }
   }
}

// --- Тест 7: мощность, пик и крест-фактор на окне 8 мс (Д_L1OC.10), (Д_L1OC.8) ---
TEST(SignalCombineL1OCPower, Test7_PowerPeakAndCrestFactor) {
   struct Reference {
      int    size;
      double powerWindow;   // P_window
      double crestFactor;
      int    fullSignCount; // отсчётов полного совпадения знаков: |Σ| = |J|
   };
   const Reference references[4] = {
      {  1, 1.000000, 1.0000, 160000  },
      {  2, 0.991250, 1.4204, 79300   },
      {  8, 0.940434, 2.9166, 8730    },
      { 24, 0.807773, 5.4508, 3707    },
   };

   for (const Reference& r : references) {
      ActiveSetL1OC activeSet(r.size);
      const double  eta = normalizationFactor(activeSet.amplitudes());
      double powerSum   = 0.0;
      double peakSum    = 0.0;
      double peakPower  = 0.0;
      int    fullSign   = 0;
      int    nonZeroQ   = 0;

      for (int n = 0; n < samplesPer8ms; ++n) {
         const std::span<const std::complex<double> > sourceSamples = activeSet.collect();
         const double sum                                           = realSum(sourceSamples);
         const OutputSample sample                                  = combine(sourceSamples, eta);
         const double power                                         = static_cast<double> (sample.real()) * sample.real()
                                                                      + static_cast<double> (sample.imag()) * sample.imag();

         powerSum += power;
         peakSum   = std::max(peakSum, std::abs(sum));
         peakPower = std::max(peakPower, power);

         if (std::abs(sum) == static_cast<double> (r.size)) {
            ++fullSign;
         }

         if (sample.imag() != 0.0f) { // e_j ≡ 1 ⇒ Q[n] = 0 тождественно (Д_L1OC.5)
            ++nonZeroQ;
         }
         activeSet.advance();
      }
      const double powerWindow = powerSum / samplesPer8ms;

      EXPECT_NEAR(powerWindow, r.powerWindow, 1e-6) << "|J|=" << r.size;
      EXPECT_DOUBLE_EQ(peakSum, static_cast<double> (r.size)) << "|J|=" << r.size;
      EXPECT_NEAR(peakPower,                          static_cast<double> (r.size), 1e-5) << "|J|=" << r.size;
      EXPECT_NEAR(std::sqrt(peakPower / powerWindow), r.crestFactor,                1e-4) << "|J|=" << r.size;
      EXPECT_EQ(fullSign, r.fullSignCount) << "|J|=" << r.size;
      EXPECT_EQ(nonZeroQ, 0) << "|J|=" << r.size;
   }
}

// --- Тест 8: структура суммы Σ_j g_j[n] при |J| = 24 (Д_L1OC.10) ---
// Сумма факторизуется через выход ЦА1, поэтому принимает лишь дискретный набор значений;
// средний квадрат меньше |J| = 24 — вклад попарной корреляции кодов (Д_L1OC.5).
TEST(SignalCombineL1OCPower, Test8_SumDistribution) {
   ActiveSetL1OC activeSet(24);
   std::set<int> values;
   double sumTotal    = 0.0;
   double squareTotal = 0.0;

   for (int n = 0; n < samplesPer8ms; ++n) {
      // Σ снимается через блок при η = 1 (случай |J| = 1, Д_L1OC.4): выход равен сумме до
      // нормировки, при |Σ| ≤ 24 приведение double→float32 точно
      const double sum = combine(activeSet.collect(), 1.0).real();

      values.insert(static_cast<int> (sum));
      sumTotal    += sum;
      squareTotal += sum * sum;
      activeSet.advance();
   }
   const std::set<int> expected = { -24, -8, -6, -2, 0, 2, 6, 8, 24 };

   EXPECT_EQ(values, expected);
   EXPECT_NEAR(sumTotal / samplesPer8ms,    -0.001150, 1e-9);
   EXPECT_NEAR(squareTotal / samplesPer8ms, 19.386550, 1e-9);
}

// --- Тест 9: порядок накопления суммы (Д_L1OC.8), (§ 2_L1OC.2) ---
TEST(SignalCombineL1OCOrder, Test9_AccumulationOrder) {
   constexpr int size = 24;

   ActiveSetL1OC activeSet(size);
   const double  eta = normalizationFactor(activeSet.amplitudes());

   std::vector<std::complex<double> > descending(size);
   std::vector<std::complex<double> > evenThenOdd(size);
   int differingReal = 0;

   for (int n = 0; n < samplesPer8ms; ++n) {
      const std::span<const std::complex<double> > ascending = activeSet.collect();

      for (int i = 0; i < size; ++i) {
         descending[static_cast<std::size_t> (i)] = ascending[static_cast<std::size_t> (size - 1 - i)];
      }
      std::size_t next = 0;

      for (int j = 2; j <= size; j += 2) { // чётные j, затем нечётные (j = 1…24)
         evenThenOdd[next++] = ascending[static_cast<std::size_t> (j - 1)];
      }

      for (int j = 1; j <= size; j += 2) {
         evenThenOdd[next++] = ascending[static_cast<std::size_t> (j - 1)];
      }
      const OutputSample reference = combine(ascending, eta);

      if (bitsOf(combine(descending, eta).real()) != bitsOf(reference.real())) {
         ++differingReal;
      }

      if (bitsOf(combine(evenThenOdd, eta).real()) != bitsOf(reference.real())) {
         ++differingReal;
      }
      activeSet.advance();
   }
   EXPECT_EQ(differingReal, 0);

   // разнесённые начальные фазы φ_{0,j} = 2π·idx/|J| ⇒ вклады комплексные (Д_L1OC.10)
   std::vector<std::complex<double> > phasors(size);

   for (int idx = 0; idx < size; ++idx) {
      const double angle = 2.0 * M_PI * idx / size;

      phasors[static_cast<std::size_t> (idx)] = { std::cos(angle), std::sin(angle) };
   }
   ActiveSetL1OC complexSet(size);
   std::vector<std::complex<double> > reversed(size);
   int differingComplex = 0;

   for (int n = 0; n < 2000; ++n) {
      const std::span<const std::complex<double> > ascending = complexSet.collect(phasors);

      for (int i = 0; i < size; ++i) {
         reversed[static_cast<std::size_t> (i)] = ascending[static_cast<std::size_t> (size - 1 - i)];
      }
      const OutputSample forward  = combine(ascending, eta);
      const OutputSample backward = combine(reversed,  eta);

      if ((bitsOf(forward.real()) != bitsOf(backward.real()))
          || (bitsOf(forward.imag()) != bitsOf(backward.imag()))) {
         ++differingComplex;
      }
      complexSet.advance();
   }
   EXPECT_GT(differingComplex, 0); // порядок фиксируется по возрастанию j (Д_L1OC.8)
}
