#pragma once
#include "glonass/types.h"
#include <array>
#include <cstddef>

namespace glonass {
namespace detail {
// Построение таблицы кода на этапе компиляции (Ч3 А.4(1), псевдокод А.9).
// РСЛОС G(x)=1+x^5+x^9; seed = все «1»; съём — phys.разряд 7.
// Порядок такта: съём s7 -> обратная связь -> сдвиг. Индексы массива 0..8 = phys 1..9.
constexpr std::array<Bit, codeLength> buildRangingCodeTable() {
   std::array<Bit, codeLength> table{};
   std::array<Bit, 9> reg{ 1, 1, 1, 1, 1, 1, 1, 1, 1 };        // phys 1..9 = «1» (А.2)

   for (int q = 0; q < codeLength; ++q) {
      table[q] = reg[6];                                       // phys 7 — съём ДО сдвига (А.1)
      const Bit feedback = static_cast<Bit> (reg[4] ^ reg[8]); // phys 5 и 9 (А.2)

      for (int j = 8; j > 0; --j) {
         reg[j] = reg[j - 1];                                  // s_j <- s_{j-1} (А.3)
      }
      reg[0] = feedback;                                       // s_1 <- feedback
   }
   return table;                                               // после 511 тактов reg == [1..1]
}
} // namespace detail

// Единый дальномерный код FDMA (Ч3 А.1–А.6). Один экземпляр на модель.
// Таблица кода — компилируемая константа (constexpr), общая для всех литер.
class RangingCode {
public:

   RangingCode() = default;

   Bit                                at(int chipIndex) const; // codeTable[chipIndex] (А.6), 0 <= chipIndex <= 510
   const std::array<Bit, codeLength> &table() const;           // диагностика/тесты

private:

   static constexpr std::array<Bit, codeLength> codeTable_ = detail::buildRangingCodeTable();
};
} // namespace glonass
