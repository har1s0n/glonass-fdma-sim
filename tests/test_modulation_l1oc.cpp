#include "glonass/modulation_l1oc.h"
#include "glonass/nav_message_l1oc.h"
#include "glonass/ranging_code_l1oc.h"
#include "sha256.h"
#include "source_l1oc.h" // сцепка А_L1OC + Б_L1OC -> Г_L1OC (SourceL1OC), общая для тестов L1OC

#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <cstdint>
#include <set>
#include <string>

using namespace glonass;
using namespace testutil;

namespace {
constexpr int samplesPer24ms = 480000; // 6 символов СК, 12 полусимволов ОК1 (Г_L1OC.10)
constexpr double kEpsPhasor  = 1e-9;   // допуск координат (§ 0.4, уровень Блока В)

// Варианты вектора для проверки его различающей силы (Г_L1OC.10).
struct Variant {
   bool noMeander = false; // МП не накладывается на пилотную компоненту
   bool noOverlay = false; // ОК1 не накладывается на информационную компоненту
   bool swapSigma = false; // обратное соглашение о порядке компонент
};

// Сериализация биполярной последовательности: «1» при g = +1, «0» при g = -1, по возрастанию n
// (Г_L1OC.10). Знак снимается с координаты I_j при единичном фазоре: g_j = Re u_j.
std::string bipolarString(int j, int sampleCount, Variant variant = {}) {
   SourceL1OC  source(j);
   std::string bits;

   bits.reserve(static_cast<std::size_t> (sampleCount));

   for (int n = 0; n < sampleCount; ++n) {
      const Bit componentSelect = variant.swapSigma
                                  ? static_cast<Bit> (source.code().componentSelect() ^ 1)
                                  : source.code().componentSelect();
      const Bit meanderSymbol = variant.noMeander ? Bit{ 0 } : source.code().meanderSymbol();
      const Bit overlaySymbol = variant.noOverlay ? Bit{ 0 } : source.message().overlaySymbol();

      const Bit multiplexedBit = multiplexL1OC(source.code().codeBitD(), source.code().codeBitP(),
                                               meanderSymbol, componentSelect,
                                               source.message().convSymbol(), overlaySymbol);

      bits.push_back(modulateL1OC(multiplexedBit, kUnitPhasor, 1.0).real() > 0.0 ? '1' : '0');
      source.step();
   }
   return bits;
}

int differingSamples(const std::string& a, const std::string& b) {
   int count = 0;

   for (std::size_t i = 0; i < a.size(); ++i) {
      if (a[i] != b[i]) {
         ++count;
      }
   }
   return count;
}
} // namespace

// --- Тест 1: таблица истинности информационной компоненты, П_d = ДК_L1OCd XOR ОК1 XOR СК ---
// (Г_L1OC.1), (Г_L1OC.4); контрольный пример Г_L1OC.10, левая таблица (8 строк).
TEST(ModulationL1OCTruthTable, Test1_InformationComponent) {
   struct Row {
      Bit    codeBitD, overlaySymbol, convSymbol;
      Bit    modulationBit;
      double sourceI;
   };
   const Row rows[8] = {
      { 0, 0, 0, 0, +1.0 },
      { 0, 0, 1, 1, -1.0 },
      { 0, 1, 0, 1, -1.0 },
      { 0, 1, 1, 0, +1.0 },
      { 1, 0, 0, 1, -1.0 },
      { 1, 0, 1, 0, +1.0 },
      { 1, 1, 0, 0, +1.0 },
      { 1, 1, 1, 1, -1.0 },
   };

   for (const Row& r : rows) {
      // sigma = 0 -> передаётся информационная компонента; входы пилотной заведомо ненулевые
      const Bit multiplexedBit = multiplexL1OC(r.codeBitD, 1, 1, 0, r.convSymbol, r.overlaySymbol);

      EXPECT_EQ(multiplexedBit, r.modulationBit)
         << "c_d=" << int(r.codeBitD) << " o=" << int(r.overlaySymbol) << " b=" << int(r.convSymbol);

      const std::complex<double> u = modulateL1OC(multiplexedBit, kUnitPhasor, 1.0);

      EXPECT_DOUBLE_EQ(u.real(), r.sourceI)
         << "c_d=" << int(r.codeBitD) << " o=" << int(r.overlaySymbol) << " b=" << int(r.convSymbol);
      EXPECT_DOUBLE_EQ(u.imag(), 0.0);
   }
}

// --- Тест 2: таблица истинности пилотной компоненты, П_p = ДК_L1OCp XOR МП ---
// (Г_L1OC.2), (Г_L1OC.4); контрольный пример Г_L1OC.10, правая таблица (4 строки).
TEST(ModulationL1OCTruthTable, Test2_PilotComponent) {
   struct Row {
      Bit    codeBitP, meanderSymbol;
      Bit    modulationBit;
      double sourceI;
   };
   const Row rows[4] = {
      { 0, 0, 0, +1.0 },
      { 0, 1, 1, -1.0 },
      { 1, 0, 1, -1.0 },
      { 1, 1, 0, +1.0 },
   };

   for (const Row& r : rows) {
      // sigma = 1 -> передаётся пилотная компонента; входы информационной заведомо ненулевые
      const Bit multiplexedBit = multiplexL1OC(1, r.codeBitP, r.meanderSymbol, 1, 1, 1);

      EXPECT_EQ(multiplexedBit, r.modulationBit)
         << "c_p=" << int(r.codeBitP) << " мп=" << int(r.meanderSymbol);

      const std::complex<double> u = modulateL1OC(multiplexedBit, kUnitPhasor, 1.0);

      EXPECT_DOUBLE_EQ(u.real(), r.sourceI)
         << "c_p=" << int(r.codeBitP) << " мп=" << int(r.meanderSymbol);
      EXPECT_DOUBLE_EQ(u.imag(), 0.0);
   }
}

// --- Тест 3: уплотнение есть ВЫБОР компоненты, а не сложение (Г_L1OC.3, [ИКД-L1OC] 2.1.4) ---
// При sigma = 0 выход не зависит от входов пилотной компоненты, при sigma = 1 — от входов
// информационной. Перебор всех 64 комбинаций шести входных бит.
TEST(ModulationL1OCMultiplexer, Test3_SelectionIsChoiceNotSum) {
   for (Bit codeBitD = 0; codeBitD <= 1; ++codeBitD) {
      for (Bit convSymbol = 0; convSymbol <= 1; ++convSymbol) {
         for (Bit overlaySymbol = 0; overlaySymbol <= 1; ++overlaySymbol) {
            // информационная компонента передаётся: значение постоянно по входам пилотной
            const Bit expectedD = multiplexL1OC(codeBitD, 0, 0, 0, convSymbol, overlaySymbol);

            for (Bit codeBitP = 0; codeBitP <= 1; ++codeBitP) {
               for (Bit meanderSymbol = 0; meanderSymbol <= 1; ++meanderSymbol) {
                  EXPECT_EQ(multiplexL1OC(codeBitD, codeBitP, meanderSymbol, 0,
                                          convSymbol, overlaySymbol), expectedD)
                     << "sigma=0, c_p=" << int(codeBitP) << " мп=" << int(meanderSymbol);
               }
            }
         }
      }
   }

   for (Bit codeBitP = 0; codeBitP <= 1; ++codeBitP) {
      for (Bit meanderSymbol = 0; meanderSymbol <= 1; ++meanderSymbol) {
         // пилотная компонента передаётся: значение постоянно по входам информационной
         const Bit expectedP = multiplexL1OC(0, codeBitP, meanderSymbol, 1, 0, 0);

         for (Bit codeBitD = 0; codeBitD <= 1; ++codeBitD) {
            for (Bit convSymbol = 0; convSymbol <= 1; ++convSymbol) {
               for (Bit overlaySymbol = 0; overlaySymbol <= 1; ++overlaySymbol) {
                  EXPECT_EQ(multiplexL1OC(codeBitD, codeBitP, meanderSymbol, 1,
                                          convSymbol, overlaySymbol), expectedP)
                     << "sigma=1, c_d=" << int(codeBitD) << " b=" << int(convSymbol)
                     << " o=" << int(overlaySymbol);
               }
            }
         }
      }
   }
}

// --- Тест 4: биполярный символ как переключатель знака (Г_L1OC.8) ---
// При П = 1 инвертируются обе координаты фазора (поворот фазы на pi); знак нулевой координаты
// инвертируется в -0,0.
TEST(ModulationL1OCSign, Test4_SignSwitchAndMinusZero) {
   const std::complex<double> e{ 0.6, 0.8 };                // нетривиальный фазор, |e| = 1

   const std::complex<double> u0 = modulateL1OC(0, e, 1.0); // g = +1 -> +e_j

   EXPECT_DOUBLE_EQ(u0.real(), +0.6);
   EXPECT_DOUBLE_EQ(u0.imag(), +0.8);

   const std::complex<double> u1 = modulateL1OC(1, e, 1.0); // g = -1 -> -e_j

   EXPECT_DOUBLE_EQ(u1.real(), -0.6);
   EXPECT_DOUBLE_EQ(u1.imag(), -0.8);

   // +-0,0 при единичном фазоре: Im u_j = -0,0 (численно == 0,0, знаковый бит инвертирован)
   const std::complex<double> unitFlip = modulateL1OC(1, kUnitPhasor, 1.0);

   EXPECT_DOUBLE_EQ(unitFlip.real(), -1.0);
   EXPECT_DOUBLE_EQ(unitFlip.imag(), 0.0);
   EXPECT_TRUE(std::signbit(unitFlip.imag()));

   const std::complex<double> unitKeep = modulateL1OC(0, kUnitPhasor, 1.0);

   EXPECT_DOUBLE_EQ(unitKeep.real(), +1.0);
   EXPECT_FALSE(std::signbit(unitKeep.imag()));
}

// --- Тест 5: амплитуда источника A_j применяется в блоке Г_L1OC (Г_L1OC.5)-(Г_L1OC.7) ---
TEST(ModulationL1OCAmplitude, Test5_MagnitudeEqualsAmplitude) {
   const std::complex<double> e{ 0.6, 0.8 };
   constexpr double amplitude = 2.0;

   const std::complex<double> u = modulateL1OC(0, e, amplitude);

   EXPECT_DOUBLE_EQ(u.real(), amplitude * 0.6);
   EXPECT_DOUBLE_EQ(u.imag(), amplitude * 0.8);
   EXPECT_NEAR(std::abs(u), amplitude, kEpsPhasor * amplitude); // |u_j| = A_j

   const std::complex<double> uFlip = modulateL1OC(1, e, amplitude);

   EXPECT_DOUBLE_EQ(uFlip.real(), -amplitude * 0.6);
   EXPECT_DOUBLE_EQ(uFlip.imag(), -amplitude * 0.8);
   EXPECT_NEAR(std::abs(uFlip), amplitude, kEpsPhasor * amplitude);
}

// --- Тест 6: сквозная прогрессия А_L1OC + Б_L1OC -> Г_L1OC (j = 1, ЦИ нулевая, n0 = 0) ---
// Контрольные точки — начала чиповых интервалов уплотнения и переходы полусимвола МП
// (Г_L1OC.10, раздел «Прогрессия»).
TEST(ModulationL1OCEndToEnd, Test6_ProgressionCheckpoints) {
   struct Checkpoint {
      int    n;
      int    multiplexChipIndex;
      Bit    componentSelect, meanderSymbol;
      Bit    codeBitD, codeBitP, convSymbol, overlaySymbol;
      Bit    multiplexedBit;
      double sourceI;
   };
   const Checkpoint checkpoints[8] = {
      {  0, 0, 0, 0, 1, 0, 0, 0, 1, -1.0 },
      { 10, 0, 0, 1, 1, 0, 0, 0, 1, -1.0 },
      { 20, 1, 1, 0, 1, 0, 0, 0, 0, +1.0 },
      { 30, 1, 1, 1, 1, 0, 0, 0, 1, -1.0 },
      { 40, 2, 0, 0, 0, 0, 0, 0, 0, +1.0 },
      { 59, 3, 1, 0, 0, 0, 0, 0, 0, +1.0 },
      { 79, 4, 0, 0, 0, 1, 0, 0, 0, +1.0 },
      { 98, 5, 1, 0, 0, 1, 0, 0, 1, -1.0 },
   };

   SourceL1OC  source(1);
   std::size_t next = 0;

   for (int n = 0; n <= checkpoints[7].n; ++n) {
      if ((next < 8) && (n == checkpoints[next].n)) {
         const Checkpoint& c = checkpoints[next];

         EXPECT_EQ(source.code().multiplexChipIndex(), c.multiplexChipIndex) << "n=" << n;
         EXPECT_EQ(source.code().componentSelect(),    c.componentSelect) << "n=" << n;
         EXPECT_EQ(source.code().meanderSymbol(),      c.meanderSymbol) << "n=" << n;
         EXPECT_EQ(source.code().codeBitD(),           c.codeBitD) << "n=" << n;
         EXPECT_EQ(source.code().codeBitP(),           c.codeBitP) << "n=" << n;
         EXPECT_EQ(source.message().convSymbol(),      c.convSymbol) << "n=" << n;
         EXPECT_EQ(source.message().overlaySymbol(),   c.overlaySymbol) << "n=" << n;
         EXPECT_EQ(source.multiplexedBit(),            c.multiplexedBit) << "n=" << n;

         const std::complex<double> u = source.sourceSample();

         EXPECT_DOUBLE_EQ(u.real(), c.sourceI) << "n=" << n; // u_j = g_j при A_j = 1, phi_0 = 0
         EXPECT_DOUBLE_EQ(u.imag(), 0.0) << "n=" << n;
         ++next;
      }
      source.step();
   }
   ASSERT_EQ(next, 8u);
}

// --- Тест 7: границы символа ОК1 и символа СК (Fs/R_ок = 40 000; Fs/R_с = 80 000) ---
// Контрольный пример Г_L1OC.10, раздел «Границы ОК1 и символа СК»; n = 159 999 — последний
// отсчёт периода замыкания кодовой фазы.
TEST(ModulationL1OCEndToEnd, Test7_OverlayAndSymbolBoundaries) {
   struct Checkpoint {
      int    n, multiplexChipIndex, convSymbolIndex;
      Bit    componentSelect, meanderSymbol;
      Bit    codeBitD, codeBitP, convSymbol, overlaySymbol;
      Bit    multiplexedBit;
      double sourceI;
   };
   const Checkpoint checkpoints[5] = {
      {  39999, 2045, 0, 1, 1, 1, 1, 0, 0, 0, +1.0 },
      {  40000, 2046, 0, 0, 0, 1, 1, 0, 1, 0, +1.0 },
      {  79999, 4091, 0, 1, 1, 1, 1, 0, 1, 0, +1.0 },
      {  80000, 4092, 1, 0, 0, 1, 1, 0, 0, 1, -1.0 },
      { 159999, 8183, 1, 1, 1, 1, 0, 0, 1, 1, -1.0 },
   };

   SourceL1OC  source(1);
   std::size_t next = 0;

   for (int n = 0; n <= checkpoints[4].n; ++n) {
      if ((next < 5) && (n == checkpoints[next].n)) {
         const Checkpoint& c = checkpoints[next];

         EXPECT_EQ(source.code().multiplexChipIndex(), c.multiplexChipIndex) << "n=" << n;
         EXPECT_EQ(source.code().componentSelect(),    c.componentSelect) << "n=" << n;
         EXPECT_EQ(source.code().meanderSymbol(),      c.meanderSymbol) << "n=" << n;
         EXPECT_EQ(source.code().codeBitD(),           c.codeBitD) << "n=" << n;
         EXPECT_EQ(source.code().codeBitP(),           c.codeBitP) << "n=" << n;
         EXPECT_EQ(source.message().convSymbolIndex(), c.convSymbolIndex) << "n=" << n;
         EXPECT_EQ(source.message().convSymbol(),      c.convSymbol) << "n=" << n;
         EXPECT_EQ(source.message().overlaySymbol(),   c.overlaySymbol) << "n=" << n;
         EXPECT_EQ(source.multiplexedBit(),            c.multiplexedBit) << "n=" << n;
         EXPECT_DOUBLE_EQ(source.sourceSample().real(), c.sourceI) << "n=" << n;
         ++next;
      }
      source.step();
   }
   ASSERT_EQ(next, 5u);
}

// --- Тест 8: SHA-256 биполярной уплотнённой последовательности, окно 8 мс, и баланс компонент ---
// Контрольный пример Г_L1OC.10: окно 160 000 отсчётов (период замыкания кодовой фазы);
// равенство мощностей компонент ([ИКД-L1OC] 2.1.1) — равное число чиповых интервалов M/2.
TEST(ModulationL1OCVector, Test8_Sha256Over8msAndComponentBalance) {
   struct Reference {
      int         satellite;
      const char* sha256;
   };
   const Reference references[3] = {
      {  0, "48119cd5cd1d802ab5b14abc188993adc827fc63cc764a4385be5c751e05ffb8" },
      {  1, "62e36417a02e6a82e2b3362171bdf6365a7e0715622f829c68175d9bac3c4f4b" },
      { 63, "db4afa27f37a1368135befb8cbb69cec8f503be46fba656d1a8839de45d4a001" },
   };

   for (const Reference& r : references) {
      const std::string bits = bipolarString(r.satellite, samplesPer8ms);

      ASSERT_EQ(bits.size(), static_cast<std::size_t> (samplesPer8ms));
      EXPECT_EQ(testutil::Sha256::hexOf(bits), r.sha256) << "j=" << r.satellite;
   }

   // баланс компонент на периоде замыкания: отсчёты 80 000 / 80 000, чиповые интервалы 4092 / 4092
   SourceL1OC source(1);
   int samplesD = 0, samplesP = 0;
   std::set<int> chipsD, chipsP;

   for (int n = 0; n < samplesPer8ms; ++n) {
      if (source.code().componentSelect() == 0) {
         ++samplesD;
         chipsD.insert(source.code().multiplexChipIndex());
      } else {
         ++samplesP;
         chipsP.insert(source.code().multiplexChipIndex());
      }
      source.step();
   }
   EXPECT_EQ(samplesD,      80000);
   EXPECT_EQ(samplesP,      80000);
   EXPECT_EQ(chipsD.size(), static_cast<std::size_t> (multiplexPeriod / 2));
   EXPECT_EQ(chipsP.size(), static_cast<std::size_t> (multiplexPeriod / 2));
}

// --- Тест 9: SHA-256 биполярной последовательности, окно 24 мс (Г_L1OC.10) ---
// 480 000 отсчётов = 6 символов СК и 12 полусимволов ОК1: окно охватывает смену b_j и o_j.
TEST(ModulationL1OCVector, Test9_Sha256Over24ms) {
   struct Reference {
      int         satellite;
      const char* sha256;
   };
   const Reference references[3] = {
      {  0, "ce83302cb333799355cec901c4677b6772017479ba880a6918793d854fb2da84" },
      {  1, "756cd667b063f9f6b208a4c6cdb70337f36308f09166d7f4f97a28eb74fa774e" },
      { 63, "1b362a7d5edbdbf0ed7081d7ee83d71621e5500e7e9d393f8f60889efa9ffc80" },
   };

   for (const Reference& r : references) {
      const std::string bits = bipolarString(r.satellite, samplesPer24ms);

      ASSERT_EQ(bits.size(), static_cast<std::size_t> (samplesPer24ms));
      EXPECT_EQ(testutil::Sha256::hexOf(bits), r.sha256) << "j=" << r.satellite;
   }
}

// --- Тест 10: различающая сила контрольного вектора (Г_L1OC.10) ---
// Снятие МП, снятие ОК1 и обратное соглашение о порядке компонент дают иную последовательность
// с заданным числом различающихся отсчётов и заданным отпечатком.
TEST(ModulationL1OCVector, Test10_DiscriminatingPower) {
   const std::string base = bipolarString(1, samplesPer8ms);

   struct Case {
      const char* name;
      Variant     variant;
      int         differing;
      const char* sha256;
   };
   const Case cases[3] = {
      { "без МП",         { true,  false, false }, 40000,
        "105fdf580c04088c6d617aaa34f0efe2dc2f79b7afc1fbe844e2e8cd7b454f02" },
      { "без ОК1",        { false, true,  false }, 40000,
        "2261e4a3e75a094f7882ab1717a95a0c16ea325cf4388d343fe9247c80ea62b5" },
      { "обратный σ",     { false, false, true  }, 79974,
        "661f20ab112ebd19c930396e28ada7e1fc5f52d6040420a0f54de374c96f4015" },
   };

   for (const Case& c : cases) {
      const std::string variantBits = bipolarString(1, samplesPer8ms, c.variant);

      EXPECT_NE(variantBits, base) << c.name;
      EXPECT_EQ(differingSamples(variantBits, base),  c.differing) << c.name;
      EXPECT_EQ(testutil::Sha256::hexOf(variantBits), c.sha256) << c.name;
   }
}

// --- Тест 11: вклад источника при phi_0 = 0 и phi_0 = pi/2 (Г_L1OC.10, раздел «Вклад источника») ---
// При f0 = f_L1OC остаточная частота Δf = 0, фазор постоянен (§ 1, (1.7)): u_j = A_j*g_j*e^{i*phi_0}.
// Первые 16 отсчётов при j = 1: g_j = -1.
TEST(ModulationL1OCSourceSample, Test11_ZeroAndQuarterInitialPhase) {
   const std::complex<double> quarterPhasor{ std::cos(M_PI / 2.0), std::sin(M_PI / 2.0) };

   SourceL1OC source(1);

   for (int n = 0; n < 16; ++n) {
      const Bit multiplexedBit = source.multiplexedBit();

      EXPECT_EQ(multiplexedBit, 1) << "n=" << n; // g_j = -1 на первых 16 отсчётах

      const std::complex<double> uZero = modulateL1OC(multiplexedBit, kUnitPhasor, 1.0);

      EXPECT_DOUBLE_EQ(uZero.real(), -1.0) << "n=" << n; // I = -1
      EXPECT_DOUBLE_EQ(uZero.imag(), 0.0) << "n=" << n;  // Q = 0

      const std::complex<double> uQuarter = modulateL1OC(multiplexedBit, quarterPhasor, 1.0);

      EXPECT_NEAR(uQuarter.real(), 0.0,  kEpsPhasor) << "n=" << n; // I = 0
      EXPECT_NEAR(uQuarter.imag(), -1.0, kEpsPhasor) << "n=" << n; // Q = -1
      source.step();
   }
}

// --- Тест 12: блок безынерционный — состояний нет, съём не изменяет источники ---
// (Г_L1OC.3), (Г_L1OC.6): выход на n — функция только текущих входов; значение устойчиво до
// продвижения состояний блоков А_L1OC и Б_L1OC (двухфазная дисциплина § 2_L1OC.2).
TEST(ModulationL1OCCombinational, Test12_OutputsStableUntilStep) {
   SourceL1OC source(1);

   for (int n = 0; n < 25; ++n) { // интервал охватывает границы чипов уплотнения m = 0…1
      const std::uint64_t codePhase   = source.code().codePhaseAccumulator();
      const std::uint64_t symbolPhase = source.message().symbolPhaseAccumulator();
      const Bit first                 = source.multiplexedBit();

      for (int repeat = 0; repeat < 3; ++repeat) {
         EXPECT_EQ(source.multiplexedBit(), first) << "n=" << n;
         EXPECT_DOUBLE_EQ(source.sourceSample().real(), first ? -1.0 : +1.0) << "n=" << n;
      }

      // повторные вычисления вклада не продвинули ни один аккумулятор
      EXPECT_EQ(source.code().codePhaseAccumulator(),      codePhase) << "n=" << n;
      EXPECT_EQ(source.message().symbolPhaseAccumulator(), symbolPhase) << "n=" << n;
      source.step();
      EXPECT_NE(source.code().codePhaseAccumulator(), codePhase) << "n=" << n;
   }
}
