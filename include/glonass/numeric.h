#include <cstdint>

namespace glonass {
// (a*b) mod m без переполнения 64 бит — двоичное умножение по модулю
// Покрывает член привязки n0*R_c в А.4(2).
// Вызывается один раз на литеру при init
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

// Целочисленное деление с округлением к ближайшему, половина — ОТ нуля (Ч3 В.2 / поз.20).
// den > 0. Числитель Δf_k·2^B < 2^54 умещается в int64 (В.8) => вещественное деление не нужно
// Покрывает Δθ_k в В.4.
inline std::int64_t roundDivHalfAwayFromZero(std::int64_t num, std::int64_t den) noexcept {
   const std::int64_t quot     = num / den;
   const std::int64_t rem      = num % den;
   const std::int64_t twiceRem = 2 * (rem < 0 ? -rem : rem);

   if (twiceRem >= den) { // |остаток| >= половины => округляем ОТ нуля
      return quot + (num >= 0 ? 1 : -1);
   }
   return quot;
}

// Евклидов остаток x − M·floor(x/M) в [0, M) (Ч3 В.4, mod_E). M > 0.
// Корректен при отрицательном приведённом Δθ_k (хранится 2^B − |Δθ_k|). Покрывает Δθ_k, Θ_k[0] в В.4.
inline std::int64_t euclideanModulo(std::int64_t x, std::int64_t M) noexcept {
   const std::int64_t r = x % M;

   return r < 0 ? r + M : r;
}
} // namespace glonass
