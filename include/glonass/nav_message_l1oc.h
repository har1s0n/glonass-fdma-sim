#ifndef NAV_MESSAGE_L1OC_H
#define NAV_MESSAGE_L1OC_H

#include <array>
#include <cstdint>
#include <functional>

#include "glonass/types.h"

namespace glonass {
// Блок Б_L1OC. Навигационное сообщение L1OC
// Один экземпляр на НКА j: строка b_line, регистр свёрточного кодера, индекс символа и фаза
// символа СК. Пилотная компонента сообщения не несёт ([ИКД-L1OC] 2.1.3): блок относится только
// к компоненте L1OCd. Наложение символов на дальномерный код выполняется в блоке Г_L1OC.

// Тип строки (§ 0.1 поз.37): им определяются длина строки n_с, образующий полином ЦК и число
// проверочных бит (Б_L1OC.11). Числовые значения согласованы со сравнением «lineType != 2» Ч3.
enum class LineTypeL1OC { normal = 0, anomalous1 = 1, anomalous2 = 2 };

int lineInfoBits(LineTypeL1OC lineType); // бит ЦИ строки: 222 / 97 / 339 ([ИКД-L1OC] табл. 4.2, 4.3, 4.5)
int lineBits(LineTypeL1OC lineType);     // n_с — бит строки: 250 / 125 / 375 (поз.37; Б_L1OC.9)

// Содержание строки, задаваемое слоем содержания (Б_L1OC.11): тип строки и информационные биты ЦИ.
// Значимы первые lineInfoBits(lineType) элементов payloadCI, остальные не читаются.
struct LineContentL1OC {
   LineTypeL1OC                        lineType = LineTypeL1OC::normal;
   std::array<Bit, maxPayloadBitsL1OC> payloadCI{};
};

// Поставщик содержания строки ℓ потока (Ч3 Б_L1OC.9: payloadOfLineL1OC + lineTypeOf).
// Слой содержания / тестовый паттерн (Б_L1OC.11).
using PayloadProviderL1OC = std::function<LineContentL1OC (std::int64_t lineIndex)>;

// Состояние регистра свёрточного кодера S[1..6] (Б_L1OC.3); индекс массива 0…5 — триггеры 1…6.
using ConvStateL1OC = std::array<Bit, 6>;

// Результат построителя строки (контракт Б_L1OC.4(2)): символы СК + выходное состояние регистра СК
struct BuiltLineL1OC {
   std::array<Bit, maxLineSymbolsL1OC> lineSymbols{};  // значимы первые lineLength символов
   int                                 lineLength = 0; // L_с = 2·n_с ∈ {250, 500, 750}
   ConvStateL1OC                       convStateOut{}; // S после последнего бита строки
};

// Построитель строки (Ч3 Б_L1OC.4, псевдокод Б_L1OC.9): информационный блок «СМВ + ЦИ» ->
// циклический код (Б_L1OC.1), (Б_L1OC.2) -> свёрточный код (133,171) (Б_L1OC.3)–(Б_L1OC.6).
// Состояние регистра СК непрерывно между строками, сброс не выполняется (поз.41).
BuiltLineL1OC buildLineL1OC(const LineContentL1OC& lineContent,
                            const ConvStateL1OC&   convStateIn);

// Потоковый генератор навигационного сообщения L1OC — состояние НА НКА (Ч3 Б_L1OC.3–Б_L1OC.10).
class NavMessageL1OC {
public:

   NavMessageL1OC() = default;

   // Инициализация от n0 (Ч3 Б_L1OC.4(4)/Б_L1OC.9, InitMessageAtSampleL1OC): lineIndex/w/P_s[0]
   // из Z0 = n0*R_с для регулярного потока 2-секундных строк; S_j = 0 (поз.41); строит текущую
   // строку. Предусл.: n0 >= 0, Fs >= R_с, первая строка потока — нормальная.
   void initMessageAtSampleL1OC(SampleIndex         globalStartSample, // n0 >= 0
                                std::int64_t        sampleRate,        // Fs
                                PayloadProviderL1OC payloadOfLineL1OC);

   // --- фаза 1: съём выходов, состояние не изменяется (Б_L1OC.7) ---
   Bit                  convSymbol() const;      // b_j[n] = b_line[w_j[n]] (Б_L1OC.8)
   Bit                  overlaySymbol() const;   // o_j[n] = ⌊2·P_{s,j}[n]/Fs⌋ (Б_L1OC.9)

   int                  convSymbolIndex() const; // w_j[n] ∈ {0…L_{с,j}−1}
   std::int64_t         lineIndex() const;       // № строки потока от t_опор
   const ConvStateL1OC &convStateOut() const;    // состояние регистра СК после текущей строки
   int                  lineLength() const;      // L_{с,j}
   std::uint64_t        symbolPhaseAccumulator() const;

   // --- фаза 2: обновление состояния и процедура границы строки (Б_L1OC.10, Б_L1OC.8) ---
   void                 step();

private:

   std::array<Bit, maxLineSymbolsL1OC> lineSymbols_{}; // b_line текущей строки
   PayloadProviderL1OC payloadOfLineL1OC_{};
   ConvStateL1OC convEncoderState_{};                  // S_j[1..6] на входе текущей строки
   ConvStateL1OC convStateOut_{};                      // S_j[1..6] после текущей строки
   std::int64_t sampleRate_              = 0;
   std::int64_t lineIndex_               = 0;
   int convSymbolIndex_                  = 0;
   int lineLength_                       = 0;
   std::uint64_t symbolPhaseAccumulator_ = 0;
};
} // namespace glonass

#endif // NAV_MESSAGE_L1OC_H
