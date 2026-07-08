#include <gtest/gtest.h>
#include <cmath>
#include <complex>
#include <cstdint>

#include "glonass/modulation.h"
#include "glonass/ranging_code.h"
#include "glonass/code_phase.h"
#include "glonass/nav_message.h"
#include "glonass/carrier_nco.h"

using namespace glonass;

namespace {
constexpr double kEpsNco = 1e-9; // допуск I/Q (§0.4), наследуется от Блока В (eps_NCO)

// Нулевой payload76: содержание строки — все «0».
std::array<Bit, 76> zeroPayload(std::int64_t /*lineIndex*/) {
   return std::array<Bit, 76>{};
}
} // namespace

// --- Тест 1: GOLDEN-таблица ФМн, e_k=1+j0, A_k=1 — точное совпадение +-1 ---
// Проверяет mu=c^b, g=1-2mu и знак u_k при тривиальном фазоре (логически точно).
TEST(ModulationGolden, Test1_BpskTruthTable) {
   const std::complex<double> e{ 1.0, 0.0 }; // carrier = 1 + j0 (Delta f = 0)

   struct Row { Bit c, b; double uI; };
   const Row rows[4] = {
      { 0, 0, +1.0 }, // mu=0 -> g=+1
      { 0, 1, -1.0 }, // mu=1 -> g=-1
      { 1, 0, -1.0 }, // mu=1 -> g=-1
      { 1, 1, +1.0 }, // mu=0 -> g=+1
   };

   for (const Row& r : rows) {
      const std::complex<double> u = modulate(r.c, r.b, e, 1.0);
      EXPECT_DOUBLE_EQ(u.real(), r.uI) << "c=" << int(r.c) << " b=" << int(r.b);
      EXPECT_DOUBLE_EQ(u.imag(), 0.0) << "c=" << int(r.c) << " b=" << int(r.b);
   }
}

// --- Тест 2: нетривиальный фазор (связь В->Г), L1 k=+6, n=1 ---
// Переключатель знака инвертирует координаты фазора: mu=1 -> u_k = -e_k = e_k*e^{j*pi}.
TEST(ModulationPhasor, Test2_SignSwitchInvertsBothComponents) {
   CarrierNco nco;

   nco.init(0, 16'368'000, carrierFreq(Band::L1OF, 6), 1'601'718'750, 0.0);
   nco.step();                                   // на n=1
   const std::complex<double> e = nco.carrier(); // e_k[1] (В.10)

   // Сверка фазора с эталоном В.10.
   EXPECT_NEAR(e.real(), 0.166492439, kEpsNco);
   EXPECT_NEAR(e.imag(), 0.986042731, kEpsNco);

   // mu=0 (c=b=0): u_k = +e_k.
   const std::complex<double> u0 = modulate(0, 0, e, 1.0);
   EXPECT_NEAR(u0.real(), 0.166492439, kEpsNco);
   EXPECT_NEAR(u0.imag(), 0.986042731, kEpsNco);

   // mu=1 (c=1,b=0): u_k = -e_k — инверсия Re и Im.
   const std::complex<double> u1 = modulate(1, 0, e, 1.0);
   EXPECT_NEAR(u1.real(), -0.166492439, kEpsNco);
   EXPECT_NEAR(u1.imag(), -0.986042731, kEpsNco);
}

// --- Тест 3: сквозной A-Г на реальных Блоках А,Б,В (одна литера k=0) ---
// f0=f_k => Delta f=0 => e_k == 1+j0; ручная сшивка §2.3 (Фаза1 съём + modulate; Фаза2 шаги).
// Валидирует шов А+Б+В+Г до появления композита SignalSource.
TEST(ModulationEndToEnd, Test3_ThroughABV) {
   constexpr std::int64_t kFs = 4'096'000; // одна литера
   const std::int64_t     fk  = carrierFreq(Band::L1OF, 0);

   RangingCode code;                       // Блок А: таблица кода
   CodePhase   codePhase;                  // Блок А: кодовая фаза

   codePhase.init(0, kFs, 0.0);
   NavMessage message;                     // Блок Б: сообщение (payload=0, relIn=0 в init)
   message.init(0, kFs, zeroPayload);
   CarrierNco nco;                         // Блок В: несущая (f0=f_k => e_k==1+j0)
   nco.init(0, kFs, fk, fk, 0.0);

   auto expectedAt = [](std::int64_t n) -> double {
                        switch (n) {
                          case 0:     return +1.0; // q=0,   c=1,b=1,mu=0
                          case 56:    return +1.0; // q=6,   c=1,b=1,mu=0 (конец чипа 6)
                          case 57:    return -1.0; // q=7,   c=0,b=1,mu=1 (скачок pi)
                          case 40959: return +1.0; // q=510, c=1,b=1,mu=0 (конец символа j=0)
                          default:    return -1.0; // n=40960: q=0, c=1,b=0,mu=1 (скачок pi, b:1->0)
                        }
                     };

   for (std::int64_t n = 0; n <= 40960; ++n) {
      // Фаза 1 (съём, состояния неизменны) — §2.3.
      const Bit codeBit                  = code.at(codePhase.chipIndex()); // c_k (Блок А, до обновления)
      const Bit msgBit                   = message.messageBit();           // b_k (Блок Б)
      const std::complex<double> carrier = nco.carrier();                  // e_k (Блок В, до обновления)
      const std::complex<double> u       = modulate(codeBit, msgBit, carrier, 1.0);

      const bool isCheckpoint = (n == 0 || n == 56 || n == 57 || n == 40959 || n == 40960);

      if (isCheckpoint) {
         EXPECT_DOUBLE_EQ(u.real(), expectedAt(n)) << "n=" << n;
         EXPECT_DOUBLE_EQ(u.imag(), 0.0) << "n=" << n; // e_k==1 => Im==0
      }

      // Фаза 2 (обновление к n+1, после съёма всех выходов) — §2.3.
      codePhase.step();
      message.step();
      nco.step();
   }
}

// --- Тест 4: поведение +-0,0 при скачке фазы pi ---
// mu=1 на carrier=1+j0 -> Im u_k = -0,0 (числ. == 0,0; знаковый бит инвертирован).
TEST(ModulationSignedZero, Test4_MinusZeroOnPhaseFlip) {
   const std::complex<double> e{ 1.0, 0.0 };

   const std::complex<double> u1 = modulate(1, 0, e, 1.0); // mu=1 -> -e = (-1, -0,0)

   EXPECT_DOUBLE_EQ(u1.imag(), 0.0);
   EXPECT_TRUE(std::signbit(u1.imag()));                   // побитово: Im = -0,0
   EXPECT_DOUBLE_EQ(u1.real(), -1.0);

   const std::complex<double> u0 = modulate(0, 0, e, 1.0); // mu=0 -> +e = (+1, +0,0)
   EXPECT_DOUBLE_EQ(u0.imag(), 0.0);
   EXPECT_FALSE(std::signbit(u0.imag()));                  // Im = +0,0
   EXPECT_DOUBLE_EQ(u0.real(), +1.0);
}

// --- Тест 5: масштаб A_k!=1 -> |u_k| = A_k ---
// Подтверждает применение амплитуды в Блоке Г
TEST(ModulationAmplitude, Test5_MagnitudeEqualsAk) {
   CarrierNco nco;

   nco.init(0, 16'368'000, carrierFreq(Band::L1OF, 6), 1'601'718'750, 0.0);
   nco.step(); // e_k[1], |e_k| ~ 1
   const std::complex<double> e = nco.carrier();

   constexpr double A           = 2.0;
   const std::complex<double> u = modulate(0, 0, e, A); // mu=0
   EXPECT_NEAR(std::abs(u), A,            kEpsNco * A); // |u_k| = A_k*|e_k| ~ A_k
   EXPECT_NEAR(u.real(),    A * e.real(), kEpsNco);     // компоненты масштабируются на A_k
   EXPECT_NEAR(u.imag(),    A * e.imag(), kEpsNco);
}
