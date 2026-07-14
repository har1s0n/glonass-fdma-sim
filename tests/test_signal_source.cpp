#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>

#include "glonass/signal_combine.h"
#include "glonass/signal_source.h"

using namespace glonass;
using cd = std::complex<double>;

namespace {
// Пустой слой содержания (payload=0 ⇒ меандр «1010…», b[0]=1; Б.10/Д.10).
PayloadProvider zeroPayload() {
   return [](std::int64_t) {
             return std::array<Bit, 76>{};
   };
}
} // namespace


// --- нормировка суммы. |K|=2, A=(1,1) ⇒ η=1/√2; обе литеры letterSample=1+j0 ⇒
//     transmitterSample = η·(2+j0) = √2 (Д.4). ---
TEST(SignalCombineD, Test2_SumNormalization) {
   const std::array<double, 2> amplitudes{ 1.0, 1.0 };
   const double eta = normalizationFactor(amplitudes);

   EXPECT_DOUBLE_EQ(eta, 1.0 / std::sqrt(2.0));        // η = 1/√2

   const std::array<cd, 2> letterSamples{ cd{ 1.0, 0.0 }, cd{ 1.0, 0.0 } };
   const OutputSample u = combine(letterSamples, eta); // η·Σ, путь точен (e≡1, без фазора)
   EXPECT_FLOAT_EQ(u.real(), static_cast<float> (std::sqrt(2.0)));
   EXPECT_FLOAT_EQ(u.imag(), 0.0f);
}

// --- неравные амплитуды. A=(2,1) ⇒ η=1/√5 (Ч3 Д.4); A_k уже в letterSample_k из Г ⇒
//     letterSample=(2,1) ⇒ transmitterSample = η·(3+j0) = 3/√5. ---
TEST(SignalCombineD, Test3_UnequalAmplitudes) {
   const std::array<double, 2> amplitudes{ 2.0, 1.0 };
   const double eta = normalizationFactor(amplitudes);

   EXPECT_DOUBLE_EQ(eta, 1.0 / std::sqrt(5.0)); // η = 1/√5 ≈ 0,4472136

   const std::array<cd, 2> letterSamples{ cd{ 2.0, 0.0 }, cd{ 1.0, 0.0 } };
   const OutputSample u  = combine(letterSamples, eta);
   const double expected = 3.0 / std::sqrt(5.0); // ≈ 1,34164079
   EXPECT_NEAR(std::hypot(static_cast<double> (u.real()), static_cast<double> (u.imag())),
               expected, 1e-6);
   EXPECT_FLOAT_EQ(u.real(), static_cast<float> (expected));
}

// --- детерминизм порядка (Д.7/Д.8; сложение float неассоциативно). combine суммирует в
//     порядке элементов span (вызывающий подаёт по возрастанию k). 1,0 теряется рядом с 1e16
//     (ulp(1e16)≈2) ⇒ порядок влияет на бит-результат. ---
TEST(SignalCombineD, Test4_OrderDeterminism) {
   const std::array<cd, 3> ascending{ cd{ 1e16, 0.0 }, cd{ 1.0, 0.0 }, cd{ -1e16, 0.0 } };
   double referenceSameOrder = 0.0;

   referenceSameOrder += 1e16;
   referenceSameOrder += 1.0;
   referenceSameOrder += -1e16;                                  // == 0,0 (1,0 потеряна)
   const OutputSample u = combine(ascending, 1.0);
   EXPECT_EQ(u.real(), static_cast<float> (referenceSameOrder)); // бит-точно к сумме порядка span

   double referencePermuted = 0.0;                               // иной порядок ⇒ иной результат

   referencePermuted += 1e16;
   referencePermuted += -1e16;
   referencePermuted += 1.0; // == 1,0
   EXPECT_NE(static_cast<float> (referenceSameOrder), static_cast<float> (referencePermuted));
}

// --- приведение double→float32 (round-to-nearest-even, Д.9). Осевые/целые — точно;
//     √14 при Σ=14, η=1/√14 (путь редукции точен) — бит-эталон float32. ---
TEST(SignalCombineD, Test5_Float32Rounding) {
   EXPECT_EQ(combine(std::array<cd, 1>{ cd{ 1.0, 0.0 } }, 1.0).real(), 1.0f); // осевая точна

   const OutputSample b = combine(std::array<cd, 1>{ cd{ 0.1, 0.2 } }, 1.0);
   EXPECT_EQ(b.real(),                                                 0.1f); // round-to-nearest-even
   EXPECT_EQ(b.imag(),                                                 0.2f);

   std::array<cd, 14> grid;
   grid.fill(cd{ 1.0, 0.0 });
   const double eta14 =
      normalizationFactor(std::array<double, 14>{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 });
   EXPECT_EQ(combine(grid, eta14).real(), static_cast<float> (std::sqrt(14.0))); // float32(√14)
}

// --- η в опорных точках (предусловие |K|≥1, Σ A_k²>0; Д.4). ---
TEST(SignalCombineD, NormalizationReferenceValues) {
   EXPECT_DOUBLE_EQ(normalizationFactor(std::array<double, 1>{ 1.0 }), 1.0); // |K|=1 ⇒ η=1
   EXPECT_DOUBLE_EQ(normalizationFactor(std::array<double, 1>{ 2.0 }), 0.5); // η=1/√4=1/2
   EXPECT_NEAR(normalizationFactor(std::array<double, 14>{
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }),
               1.0 / std::sqrt(14.0), 1e-15);                                // Д.10
}

// ============================================================================
// SignalSource: сборка А+Б+В+Г+Д, двухфазный шаг
// ============================================================================

// --- единичная норма end-to-end. k=0, f₀=f_k ⇒ Δf=0 ⇒ e_k≡1; |K|=1, A=1 ⇒ η=1 ⇒
//     последовательность ±1+j0 в float32 (точно). Проверяет тождество при η=1, приведение
//     float32 (Д.9), двухфазный шаг (§ 2.3). Осевые ±1,0/0,±1 — точно. ---
TEST(SignalSourceEndToEnd, Test1_UnitNormEnvelope) {
   SourceConfig cfg;

   cfg.band              = Band::L1OF;
   cfg.sampleRate        = 16368000;                   // Fs (Ч3)
   cfg.referenceFreq     = carrierFreq(Band::L1OF, 0); // f₀ = f_{k=0} ⇒ Δf=0 ⇒ e≡1
   cfg.globalStartSample = 0;                          // n₀=0 (golden)
   cfg.letters           = { LetterConfig{ 0, 1.0, 0.0, 0.0, zeroPayload() } };

   SignalSource src(cfg);
   EXPECT_EQ(src.letterCount(), std::size_t{ 1 });
   EXPECT_DOUBLE_EQ(src.normalizationFactor(), 1.0); // η=1 (|K|=1, A=1)

   const OutputSample first = src.step();            // n=0: c[0]=1,b[0]=1 ⇒ μ=0 ⇒ g=+1, e=1
   EXPECT_FLOAT_EQ(first.real(), 1.0f);
   EXPECT_FLOAT_EQ(first.imag(), 0.0f);

   for (int n = 1; n < 4096; ++n) { // ±1+j0 точно на всей выборке
      const OutputSample u = src.step();
      EXPECT_TRUE(u.real() == 1.0f || u.real() == -1.0f) << "n=" << n << " I=" << u.real();
      EXPECT_EQ(u.imag(), 0.0f) << "n=" << n;
   }
   EXPECT_EQ(src.sampleIndex(), SampleIndex{ 4096 }); // n = n₀ + r
}

// --- полный сквозной SignalSource — сборка А+Б+В+Г+Д по § 2.3, полная сетка L1 |K|=14.
//     Воспроизводит контрольный пример Ч3 Д.10: n=0 ⇒ √14; n=1 ⇒ 2,475933500; Q≈0 (симметрия
//     Δf_k). Допуск ε_Σ на вещественную часть (фазор + float32). ---
TEST(SignalSourceEndToEnd, Test6_FullL1GridIcdD10) {
   SourceConfig cfg;

   cfg.band              = Band::L1OF;
   cfg.sampleRate        = 16368000;   // Fs (Ч3 Д.10)
   cfg.referenceFreq     = 1601718750; // f₀ = f_центр сетки (k=−7…+6)
   cfg.globalStartSample = 0;          // n₀=0

   for (int k = -7; k <= 6; ++k) {     // K={−7,…,+6}, |K|=14, A_k=1
      cfg.letters.push_back(LetterConfig{ k, 1.0, 0.0, 0.0, zeroPayload() });
   }

   SignalSource src(cfg);
   EXPECT_EQ(src.letterCount(), std::size_t{ 14 });
   EXPECT_NEAR(src.normalizationFactor(),       1.0 / std::sqrt(14.0), 1e-15); // η=0,267261242

   const OutputSample u0 = src.step();                                         // n=0: Σe_k=14 ⇒ √14
   EXPECT_NEAR(static_cast<double> (u0.real()), std::sqrt(14.0),       1e-5);  // 3,741657387 (Ч3 Д.10)
   EXPECT_NEAR(static_cast<double> (u0.imag()), 0.0,                   1e-5);  // Q[0]=0

   const OutputSample u1 = src.step();                                         // n=1: Σe_k=9,264094869
   EXPECT_NEAR(static_cast<double> (u1.real()), 2.475933500,           1e-5);  // 2,475933500 (Ч3 Д.10)
   EXPECT_NEAR(static_cast<double> (u1.imag()), 0.0,                   1e-5);  // |Q[1]| ≤ ε_Σ (§ 0.4)
}
