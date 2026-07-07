#ifndef NAV_MESSAGE_H
#define NAV_MESSAGE_H

#include <array>
#include <functional>
#include <cstdint>

#include "glonass/types.h"

namespace glonass {
namespace detail {
// Метка времени ПСПМВ (Ч3 Б.4(4), [ИКД] 3.3.2.2). Укороченная (31->30) M-последовательность.
// РСЛОС g(x)=1+x^3+x^5; seed = все «1»; съём — разряд 5; порядок такта: съём s5 -> ОС -> сдвиг.
// Генерируется 31 символ (полный период); в строку идут первые 30. Отбрасываемый 31-й символ = 0
// («холостой» символ поз.85 следующей строки, [ИКД] 3.3.2.2/4.3.3). Индексы reg[0..4] = разряды 1..5.
constexpr std::array<Bit, timeMarkLength> buildTimeMarkTable() {
   std::array<Bit, timeMarkLength> table{};
   std::array<Bit, 5> reg{ 1, 1, 1, 1, 1 };                    // разряды 1..5 = «1» ([ИКД] аналогично ПСПД)

   for (int m = 0; m < timeMarkLength; ++m) {                  // 30 символов (укорочение 31->30)
      table[m] = reg[4];                                       // разряд 5 — съём ДО сдвига
      const Bit feedback = static_cast<Bit> (reg[2] ^ reg[4]); // разряды 3 и 5 (g=1+x^3+x^5)

      for (int j = 4; j > 0; --j) {
         reg[j] = reg[j - 1];                                  // s_j <- s_{j-1}
      }
      reg[0] = feedback;                                       // s_1 <- feedback
   }
   return table;                                               // 31-й символ (=0) не выдаётся (укорачиваем)
}
} // namespace detail

// Поставщик 76-битного полезного поля строки l (позиции ИКД 9..84).
// Слой содержания / тестовый паттерн (Ч3 Б.11).
using PayloadProvider = std::function<std::array<Bit, 76> (std::int64_t lineIndex)>;

// Расширенный кодер Хэмминга (d=4): β1..β8 по спискам L1..L7 (Ч3 Б.4(1), [ИКД] Табл. 4.13).
// dPos[p-1] = символ позиции ИКД p (1..85). Читаются только позиции 9..85 (dPos[8..84]);
// позиции 1..8 — возвращаемая проверочная группа, на входе игнорируются. Возврат [0..7] = β1..β8.
std::array<Bit, 8> hammingParity(const std::array<Bit, 85>& dPos);

// Результат построителя строки (Ч3 Б.9): 200 бидвоичных символов + выходное состояние относительно кодера
struct BuiltLine {
   std::array<Bit, 200> lineSymbols;      // b_line[0..199] = 170 (ЦИ) + 30 (метка)
   Bit                  relativeStateOut; // a_out (= a_in, инвариант чётности)
};

// Построитель строки b_line (Ч3 Б.4/Б.9): Хэмминг -> непрерывный отн. код -> меандр («с 1») -> метка ПСПМВ.
// payload76[p-9] — бит позиции ИКД p (9..84); позиция 85 = холостой «0» (Ч3 Б.4(2), [ИКД] 3.3.2.2/4.3.3).
BuiltLine buildLine(const std::array<Bit, 76>& payload76,
                    Bit                        relativeStateIn);

// Потоковый генератор навигационного сообщения — состояние НА ЛИТЕРУ (Ч3 Б.1–Б.9).
class NavMessage {
public:

   // Инициализация от n0 (Ч3 Б.4(5)/Б.9, InitMessageAtSample): lineIndex/messageSymbolIndex/
   // messagePhaseAccumulator из Z0=n0*R_m; строит текущую строку. Предусл.: n0>=0, Fs>=R_m.
   void init(SampleIndex     globalStartSample, // n0 >= 0
             std::int64_t    sampleRate,        // Fs
             PayloadProvider payloadOfLine);    // слой содержания (Б.11)

   Bit          messageBit() const;             // b_line[messageSymbolIndex] — съём ДО обновления (Ч3 Б.5/Б.7)
   void         step();                         // фаза сообщения + событие границы строки j:199->0 (Ч3 Б.6/Б.8)

   // диагностика (тесты §7):
   int          messageSymbolIndex() const;     // j in 0..199
   std::int64_t lineIndex() const;              // № строки от t_опор
   Bit          relativeStateOut() const;       // a_out текущей строки

private:

   std::array<Bit, 200> lineSymbols_{};        // b_line текущей строки
   PayloadProvider payloadOfLine_{};           // слой содержания
   std::int64_t sampleRate_               = 0; // Fs
   std::int64_t lineIndex_                = 0; // № строки
   int messageSymbolIndex_                = 0; // j
   std::uint64_t messagePhaseAccumulator_ = 0; // P_m in 0..Fs-1
   Bit relativeStateOut_                  = 0; // a_out (= a_in следующей строки)
};
} // namespace glonass

#endif // NAV_MESSAGE_H
