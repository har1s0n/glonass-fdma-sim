#include <gtest/gtest.h>
#include <complex>
#include <cstdint>

#include "glonass/carrier_nco.h"

using namespace glonass;

namespace {
constexpr std::int64_t kFs1       = 4'096'000;     // одна литера
constexpr std::int64_t kFs2       = 16'368'000;    // полная сетка
constexpr std::int64_t kCenterL1  = 1'601'718'750; // f_центр L1 = 1602 МГц − 562,5/2 кГц (В.2)
constexpr double kEpsNco          = 1e-9;          // допуск I/Q (§0.4): значения даны до 9 знаков
constexpr std::uint64_t kTwoPow32 = std::uint64_t{ 1 } << 32;

// Локальная эталонная фаза Θ[n] = (n·Δθ) mod 2^32 — независимо от NCO (сверка шага).
std::uint32_t expectedPhase(std::uint32_t dtheta, std::int64_t n) {
   return static_cast<std::uint32_t> (
      (static_cast<std::uint64_t> (dtheta) * static_cast<std::uint64_t> (n)) % kTwoPow32);
}
} // namespace

// --- Тест 1: одна литера L1 k=0, Fs=4,096 МГц, f0=f_k, φ0=0, n0=0 => несущая-константа ---
TEST(CarrierNcoGolden, Test1_SingleLetterConstantCarrier) {
   CarrierNco nco;
   const std::int64_t fk = carrierFreq(Band::L1OF, 0);

   nco.init(0, kFs1, fk, fk, 0.0); // Δf=0 => Δθ=0
   EXPECT_EQ(nco.phaseIncrement(), 0u);

   for (std::int64_t n = 0; n < 10000; ++n) {
      EXPECT_EQ(nco.carrierPhase(), 0u) << "n=" << n;
      const std::complex<double> e = nco.carrier();
      EXPECT_EQ(e.real(),           1.0) << "n=" << n; // I ≡ +1 (точно)
      EXPECT_EQ(e.imag(),           0.0) << "n=" << n; // Q ≡ 0 (точно)
      nco.step();
   }
}

// --- Тест 2: чистый тон Δf=Fs/4 (Fs=16,368 МГц) => Δθ=2^30, период 4 ---
TEST(CarrierNcoTone, Test2_QuarterRateTone) {
   CarrierNco nco;

   nco.init(0, kFs2, kFs2 / 4, 0, 0.0);          // f0=0 (условный ноль) => Δf=Fs/4
   EXPECT_EQ(nco.phaseIncrement(), 1073741824u); // 2^30

   const std::uint32_t seq[4] = { 0u, 1073741824u, 2147483648u, 3221225472u };
   const double io[4][2]      = { { 1.0, 0.0 }, { 0.0, 1.0 }, { -1.0, 0.0 }, { 0.0, -1.0 } };

   for (int n = 0; n < 4; ++n) {
      EXPECT_EQ(nco.carrierPhase(), seq[n]) << "n=" << n;
      const std::complex<double> e = nco.carrier();
      EXPECT_DOUBLE_EQ(e.real(), io[n][0]) << "n=" << n; // осевые — логически точно (−0,0 == 0)
      EXPECT_DOUBLE_EQ(e.imag(), io[n][1]) << "n=" << n;
      nco.step();
   }
   EXPECT_EQ(nco.carrierPhase(), 0u); // Θ[4]=Θ[0]
}

// --- Тест 3: сетка L1, Fs=16,368 МГц, f0=f_центр — целочисленные Δθ (побитово) ---
TEST(CarrierNcoGrid, Test3_L1GridPhaseIncrements) {
   struct Case {
      int           k;
      std::uint32_t dtheta;
   };
   const Case cases[] = {
      { +6, 959400915u  }, // Δf=+3 656 250
      { -7, 3335566381u }, // Δf=−3 656 250 (вычет 2^32−959400915)
      {  0, 73800070u   }, // Δf=+281 250
   };

   for (const Case& c : cases) {
      CarrierNco nco;
      nco.init(0, kFs2, carrierFreq(Band::L1OF, c.k), kCenterL1, 0.0);
      EXPECT_EQ(nco.phaseIncrement(), c.dtheta) << "k=" << c.k;
   }
   EXPECT_EQ(3335566381u, static_cast<std::uint32_t> (kTwoPow32 - 959400915u)); // зеркальность Δθ
}

// --- Тест 4: фазор k=+6 (Fs=16,368 МГц, f0=f_центр) — Θ побитово, I/Q с допуском ---
TEST(CarrierNcoGrid, Test4_PhasorK6) {
   CarrierNco nco;

   nco.init(0, kFs2, carrierFreq(Band::L1OF, 6), kCenterL1, 0.0);

   const std::uint32_t th[5] = { 0u, 959400915u, 1918801830u, 2878202745u, 3837603660u };
   const double io[5][2]     = {
      {  1.000000000, 0.000000000  },
      {  0.166492439, 0.986042731  },
      { -0.944560535, 0.328337319  },
      { -0.481016814, -0.876711369 },
      {  0.784389210, -0.620268948 },
   };

   for (int n = 0; n < 5; ++n) {
      EXPECT_EQ(nco.carrierPhase(), th[n]) << "n=" << n;
      const std::complex<double> e = nco.carrier();
      EXPECT_NEAR(e.real(), io[n][0], kEpsNco) << "n=" << n;
      EXPECT_NEAR(e.imag(), io[n][1], kEpsNco) << "n=" << n;
      nco.step();
   }
}

// --- Тест 5: зеркальность k=−7, n=1 (Q обратен k=+6) ---
TEST(CarrierNcoGrid, Test5_MirrorK7) {
   CarrierNco nco;

   nco.init(0, kFs2, carrierFreq(Band::L1OF, -7), kCenterL1, 0.0);
   nco.step(); // на n=1

   EXPECT_EQ(nco.carrierPhase(), 3335566381u);
   const std::complex<double> e = nco.carrier();
   EXPECT_NEAR(e.real(), 0.166492439,  kEpsNco); // I совпадает с k=+6
   EXPECT_NEAR(e.imag(), -0.986042731, kEpsNco); // Q — противоположный знак (вращение обратно)
}

// --- Тест 6: осевые точки Θ in {0,2^30,2^31,3·2^30} — логически точно ---
TEST(CarrierNcoAxes, Test6_AxisPointsExact) {
   CarrierNco nco;

   nco.init(0, kFs2, kFs2 / 4, 0, 0.0); // Δθ=2^30 обходит все четыре оси

   const double io[4][2] = { { 1.0, 0.0 }, { 0.0, 1.0 }, { -1.0, 0.0 }, { 0.0, -1.0 } };

   for (int q = 0; q < 4; ++q) {
      const std::complex<double> e = nco.carrier();
      EXPECT_DOUBLE_EQ(e.real(), io[q][0]) << "quadrant=" << q; // −0,0 == 0
      EXPECT_DOUBLE_EQ(e.imag(), io[q][1]) << "quadrant=" << q;
      nco.step();
   }
}

// --- Тест 7: убывание по кругу для k<k_центр (k=−7) — строгое равенство uint32 ---
TEST(CarrierNcoGrid, Test7_DescendingModulusK7) {
   CarrierNco nco;

   nco.init(0, kFs2, carrierFreq(Band::L1OF, -7), kCenterL1, 0.0);

   const std::uint32_t dtheta = nco.phaseIncrement(); // 3335566381 = 2^32 − 959400915
   std::uint32_t prev         = nco.carrierPhase();   // Θ[0]=0

   for (std::int64_t n = 1; n <= 8; ++n) {
      nco.step();
      const std::uint32_t cur = nco.carrierPhase();
      EXPECT_EQ(cur, expectedPhase(dtheta, n)) << "n=" << n;          // Θ[n] побитово
      EXPECT_EQ(cur, static_cast<std::uint32_t> (prev - 959400915u)); // убывание на |Δθ| по кругу
      prev = cur;
   }
}

// --- Тест 8: непредставимая конфигурация отклоняется ---
#ifndef NDEBUG
TEST(CarrierNcoRepresentability, Test8_GridRejectedAtLowFs) {
   // Сетка L1 (k=+6, |Δf|=3,65625 МГц) при Fs=4,096 МГц: |Δf|+B_model=4,167 МГц > Fs/2=2,048 МГц.
   CarrierNco nco;

   EXPECT_DEATH(nco.init(0, kFs1, carrierFreq(Band::L1OF, 6), kCenterL1, 0.0), "");
}
#endif // ifndef NDEBUG
