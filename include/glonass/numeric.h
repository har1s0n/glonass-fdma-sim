#pragma once
#include <cstdint>

namespace glonass {
// (a*b) mod m без переполнения 64 бит — двоичное умножение по модулю
// Покрывает член привязки n0*R_c в А.4(2).
// Вызывается один раз на литеру при init — производительность нерелевантна.
inline std::uint64_t mulMod(std::uint64_t a, std::uint64_t b, std::uint64_t m) noexcept {
   a %= m;
   std::uint64_t result = 0;

   while (b != 0) {
      if (b & 1u) {
         result = (result + a) % m; // result,a < m => сумма < 2m <= 2^64
      }
      a   = (a + a) % m;            // a < m       => 2a    < 2m <= 2^64
      b >>= 1;
   }
   return result;
}
} // namespace glonass
