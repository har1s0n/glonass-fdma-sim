#include "glonass/carrier_nco.h"
#include "glonass/modulation_l1oc.h"
#include "glonass/types.h"
#include "source_l1oc.h"

#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <cstdint>

using namespace glonass;

namespace {
constexpr std::int64_t kFsL1OC   = testutil::sampleRateL1OC; // Fs = 20,0 МГц
constexpr std::int64_t kFsMinRep = 2 * modelBandwidthL1OC;   // 4 092 000 Гц — граница представимости (В.2)
constexpr double kEpsNco         = 1e-9;                     // допуск I/Q (§ 0.4): значения даны до 9 знаков
constexpr double kPi             = 3.14159265358979323846;

// Контрольные начальные фазы Θ_j[0] = round(φ_{0,j}/(2π)·2^B) при Δθ_j = 0 (В.3).
struct PhaseCase {
   const char*   name;
   double        initialPhase; // φ_{0,j}, рад
   std::uint32_t phase;        // Θ_j[0]
   double        carrierI;     // Re e_j
   double        carrierQ;     // Im e_j
};

const PhaseCase kPhaseCases[] = {
   { "0",     0.0,             0u,                      1.0,                      0.0                                                  },
   { "pi/2",  kPi / 2.0,       1073741824u,             -0.0,                     1.0                                                  },
   { "pi",    kPi,             2147483648u,             -1.0,                     -0.0                                                 },
   { "3pi/2", 3.0 * kPi / 2.0, 3221225472u,             0.0,                      -1.0                                                 },
   { "pi/4",  kPi / 4.0,       536870912u,              0.707106781,              0.707106781                                          },
   { "-pi/2", -kPi / 2.0,      3221225472u,             0.0,                      -1.0                                                 },
};
} // namespace

// --- Тест 1: параметры несущей тракта L1OC ([ИКД-L1OC] 2.1.1; § 0.1 позиции 26, 34) ---
TEST(CarrierNcoL1OCParameters, Test1_IcdValues) {
   constexpr std::int64_t baseFreq = 1'023'000; // f_b — базовая частота ([ИКД-L1OC], перечень обозначений)

   // Обе формы записи ИКД п. 2.1.1: f_L1 = 1565·f_b = 313·5,115 МГц = 1600,995 МГц
   EXPECT_EQ(carrierFreqL1OC,    1565 * baseFreq);
   EXPECT_EQ(carrierFreqL1OC,    313 * 5'115'000);
   EXPECT_EQ(carrierFreqL1OC,    1'600'995'000);

   // B_model = 2·f_T1 — главный лепесток по первым нулям (позиция 34; В.2)
   EXPECT_EQ(modelBandwidthL1OC, 2 * chipRateL1OC);
   EXPECT_EQ(modelBandwidthL1OC, 2'046'000);

   // При max|Δf| = 0 условие B_model ≤ Fs/2 даёт Fs ≥ 4,092 МГц; Fs = 20,0 МГц (позиция 35) — с запасом
   EXPECT_EQ(kFsMinRep,          4'092'000);
   EXPECT_LE(modelBandwidthL1OC, kFsL1OC / 2);

   // Тракты не смешиваются (§ 2_L1OC.1): f_L1OC не совпадает ни с одной несущей сетки L1OF
   for (int letter = -7; letter <= 6; ++letter) {
      EXPECT_NE(carrierFreq(Band::L1OF, letter), carrierFreqL1OC) << "k=" << letter;
   }
}

// --- Тест 2: f₀ = f_L1OC ⇒ Δf_j = 0 ⇒ Δθ_j = 0 при любой Fs (§ 1 (1.7); В.1, В.2) ---
TEST(CarrierNcoL1OCResidual, Test2_ZeroPhaseIncrement) {
   const std::int64_t sampleRates[] = { kFsMinRep, 16'368'000, kFsL1OC, 100'000'000 };

   for (std::int64_t sampleRate : sampleRates) {
      CarrierNco nco;

      nco.init(0, sampleRate, carrierFreqL1OC, carrierFreqL1OC, 0.0, modelBandwidthL1OC);
      EXPECT_EQ(nco.phaseIncrement(), 0u) << "Fs=" << sampleRate;
   }

   // Контраст: смещение опорной частоты на 1 Гц уже даёт ненулевое Δθ — тождество (1.7)
   // держится именно на равенстве f₀ = f_L1OC, а не на устройстве проверки.
   CarrierNco shifted;

   shifted.init(0, kFsL1OC, carrierFreqL1OC, carrierFreqL1OC - 1, 0.0, modelBandwidthL1OC);
   EXPECT_EQ(shifted.phaseIncrement(), 215u); // RoundDiv(1·2³², 20·10⁶) = 215
}

// --- Тест 3: Θ_j[0] не зависит от n₀ (В.3: MulMod(n₀, 0, 2ᴮ) = 0) ---
TEST(CarrierNcoL1OCStartSample, Test3_InitialPhaseIndependentOfStartSample) {
   const SampleIndex startSamples[] = { 0, 1, 160'000, 20'000'000, 1'000'000'000,
                                        86'400 * kFsL1OC }; // сутки в отсчётах

   for (const PhaseCase& phaseCase : kPhaseCases) {
      for (SampleIndex startSample : startSamples) {
         CarrierNco nco;

         nco.init(startSample, kFsL1OC, carrierFreqL1OC, carrierFreqL1OC,
                  phaseCase.initialPhase, modelBandwidthL1OC);
         EXPECT_EQ(nco.carrierPhase(), phaseCase.phase)
            << "phi_0j=" << phaseCase.name << ", n0=" << startSample;
      }
   }
}

// --- Тест 4: постоянство Θ_j[n] и e_j[n] на периоде замыкания кодовой фазы (В.5; § 1 (1.7)) ---
TEST(CarrierNcoL1OCConstancy, Test4_ConstantPhasorOverMultiplexPeriod) {
   CarrierNco nco;

   nco.init(0, kFsL1OC, carrierFreqL1OC, carrierFreqL1OC, 0.0, modelBandwidthL1OC);

   for (int n = 0; n < testutil::samplesPer8ms; ++n) {
      ASSERT_EQ(nco.carrierPhase(), 0u) << "n=" << n;
      const std::complex<double> carrier = nco.carrier();

      ASSERT_EQ(carrier.real(),     1.0) << "n=" << n; // I ≡ +1 (точно)
      ASSERT_EQ(carrier.imag(),     0.0) << "n=" << n; // Q ≡  0 (точно)
      nco.step();                                      // Θ_j[n+1] = Θ_j[n] + 0 (В.5)
   }
}

// --- Тест 5: Θ_j[0] и e_j для контрольных φ_{0,j} (В.3, В.4) ---
TEST(CarrierNcoL1OCInitialPhase, Test5_AxisAndDiagonalPoints) {
   for (const PhaseCase& phaseCase : kPhaseCases) {
      CarrierNco nco;

      nco.init(0, kFsL1OC, carrierFreqL1OC, carrierFreqL1OC,
               phaseCase.initialPhase, modelBandwidthL1OC);
      EXPECT_EQ(nco.carrierPhase(), phaseCase.phase) << "phi_0j=" << phaseCase.name;

      const std::complex<double> carrier = nco.carrier();

      EXPECT_NEAR(carrier.real(), phaseCase.carrierI, kEpsNco) << "phi_0j=" << phaseCase.name;
      EXPECT_NEAR(carrier.imag(), phaseCase.carrierQ, kEpsNco) << "phi_0j=" << phaseCase.name;
   }

   // φ_{0,j} = −π/2 и 3π/2 — один и тот же вычет по модулю 2ᴮ (В.3: mod_E).
   EXPECT_EQ(kPhaseCases[3].phase, kPhaseCases[5].phase);
}

// --- Тест 6: квадратурное положение φ_{0,j} = π/2 выходит точно (В.8; § 0.1 позиция 46) ---
TEST(CarrierNcoL1OCQuadrature, Test6_QuarterTurnIsExact) {
   CarrierNco nco;

   nco.init(0, kFsL1OC, carrierFreqL1OC, carrierFreqL1OC, kPi / 2.0, modelBandwidthL1OC);
   EXPECT_EQ(nco.carrierPhase(), 1073741824u); // 2³⁰: π/2 / (2π) = 0,25 представимо в double точно

   const std::complex<double> carrier = nco.carrier();

   EXPECT_EQ(carrier.real(), 0.0);            // осевая точка: логически 0
   EXPECT_TRUE(std::signbit(carrier.real())); // и это −0,0 (В.8: с учётом представления −0,0)
   EXPECT_EQ(carrier.imag(), 1.0);            // Q ≡ +1 (точно)

   // Прямое вычисление cos/sin от π/2 точного нуля не даёт: осевая точка выходит точно именно
   // благодаря целочисленной фазе и квадрантному свёртыванию (В.8).
   const double naiveI = std::cos(kPi / 2.0);

   EXPECT_NE(naiveI, 0.0);
   EXPECT_LT(std::abs(naiveI), 1e-16);
}

// --- Тест 7: |e_j| = 1 (В.11) ---
TEST(CarrierNcoL1OCMagnitude, Test7_UnitMagnitude) {
   for (const PhaseCase& phaseCase : kPhaseCases) {
      CarrierNco nco;

      nco.init(0, kFsL1OC, carrierFreqL1OC, carrierFreqL1OC,
               phaseCase.initialPhase, modelBandwidthL1OC);
      EXPECT_NEAR(std::abs(nco.carrier()), 1.0, kEpsNco) << "phi_0j=" << phaseCase.name;
   }
}

// --- Тест 8: при φ_{0,j} = 0 блок В даёт единичный фазор сцепки тестов L1OC ---
// Подтверждает, что kUnitPhasor (tests/source_l1oc.h), на котором построены контрольные примеры
// Г_L1OC.10 и Д_L1OC.10, совпадает с выходом реального блока В бит в бит.
TEST(CarrierNcoL1OCChain, Test8_MatchesUnitPhasorOfTestChain) {
   CarrierNco nco;

   nco.init(0, kFsL1OC, carrierFreqL1OC, carrierFreqL1OC, 0.0, modelBandwidthL1OC);

   const std::complex<double> carrier = nco.carrier();

   EXPECT_EQ(carrier.real(), testutil::kUnitPhasor.real());
   EXPECT_EQ(carrier.imag(), testutil::kUnitPhasor.imag());
   EXPECT_FALSE(std::signbit(carrier.real()));
   EXPECT_FALSE(std::signbit(carrier.imag()));
}

// --- Тест 9: связка В → Г_L1OC на реальном фазоре (Г_L1OC.5; § 0.1 позиция 46) ---
// u_j = A_j·g_j·e_j; g_j применяется как переключатель знака фазора (Г_L1OC.8).
TEST(CarrierNcoL1OCChain, Test9_SourceSampleWithRealCarrier) {
   CarrierNco zeroPhase;
   CarrierNco quarterPhase;

   zeroPhase.init(0, kFsL1OC, carrierFreqL1OC, carrierFreqL1OC, 0.0, modelBandwidthL1OC);
   quarterPhase.init(0, kFsL1OC, carrierFreqL1OC, carrierFreqL1OC, kPi / 2.0, modelBandwidthL1OC);

   const std::complex<double> carrierZero    = zeroPhase.carrier();
   const std::complex<double> carrierQuarter = quarterPhase.carrier();

   // φ_{0,j} = 0: вклад вещественный, u_j = g_j (эталон позиции 46)
   const std::complex<double> uZeroPlus  = modulateL1OC(0, carrierZero, 1.0);
   const std::complex<double> uZeroMinus = modulateL1OC(1, carrierZero, 1.0);

   EXPECT_EQ(uZeroPlus.real(),  1.0);
   EXPECT_EQ(uZeroPlus.imag(),  0.0);
   EXPECT_EQ(uZeroMinus.real(), -1.0);
   EXPECT_EQ(uZeroMinus.imag(), 0.0);
   EXPECT_TRUE(std::signbit(uZeroMinus.imag())); // −0,0: инверсия знака нулевой координаты (Г_L1OC.8)

   // φ_{0,j} = π/2: вклад лежит на оси Q — квадратурное положение (позиция 46)
   const std::complex<double> uQuarterPlus  = modulateL1OC(0, carrierQuarter, 1.0);
   const std::complex<double> uQuarterMinus = modulateL1OC(1, carrierQuarter, 1.0);

   EXPECT_EQ(uQuarterPlus.real(),  0.0);
   EXPECT_EQ(uQuarterPlus.imag(),  1.0);
   EXPECT_EQ(uQuarterMinus.real(), 0.0);
   EXPECT_EQ(uQuarterMinus.imag(), -1.0);

   // Амплитуда переносится на модуль вклада: |u_j| = A_j (Г_L1OC.11)
   EXPECT_NEAR(std::abs(modulateL1OC(1, carrierQuarter, 2.5)), 2.5, kEpsNco);
}

// --- Тест 10: условие представимости тракта L1OC (В.2, § 0.1 позиция 34) ---
#ifndef NDEBUG
TEST(CarrierNcoL1OCRepresentability, Test10_ModelBandwidthOfL1OC) {
   // При Δf_j = 0 условие принимает вид B_model ≤ Fs/2 ⇒ Fs ≥ 2·B_model = 4,092 МГц.
   {
      CarrierNco nco;

      nco.init(0, kFsMinRep, carrierFreqL1OC, carrierFreqL1OC, 0.0, modelBandwidthL1OC);
      EXPECT_EQ(nco.phaseIncrement(), 0u); // граница Fs = 4 092 000 проходит
   }
   {
      CarrierNco nco;

      EXPECT_DEATH(nco.init(0, kFsMinRep - 1, carrierFreqL1OC, carrierFreqL1OC,
                            0.0, modelBandwidthL1OC), "");
   }

   // Полоса FDMA (R_c, умолчание) для L1OC непригодна: она пропустила бы Fs = 3,0 МГц, при которой
   // главный лепесток L1OC лежит вне полосы Найквиста (В.2).
   {
      CarrierNco nco;

      nco.init(0, 3'000'000, carrierFreqL1OC, carrierFreqL1OC, 0.0); // B_model = codeRate — проходит
      EXPECT_EQ(nco.phaseIncrement(), 0u);
   }
   {
      CarrierNco nco;

      EXPECT_DEATH(nco.init(0, 3'000'000, carrierFreqL1OC, carrierFreqL1OC,
                            0.0, modelBandwidthL1OC), "");
   }
}
#endif // ifndef NDEBUG
