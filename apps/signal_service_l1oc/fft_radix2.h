#ifndef SERVICE_FFT_RADIX2_H
#define SERVICE_FFT_RADIX2_H

#include <cassert>
#include <cmath>
#include <complex>
#include <cstddef>
#include <numbers>
#include <utility>
#include <vector>

// Быстрое преобразование Фурье, основание 2, прореживание по времени.

namespace glonass_service {
// Преобразование на месте; длина обязана быть степенью двойки.
inline void fftRadix2(std::vector<std::complex<double> >& data) {
   const std::size_t length = data.size();

   assert((length > 0) && ((length & (length - 1)) == 0));

   // Перестановка в обратном битовом порядке
   for (std::size_t i = 1, j = 0; i < length; ++i) {
      std::size_t bit = length >> 1;

      for (; (j & bit) != 0; bit >>= 1) {
         j ^= bit;
      }
      j |= bit;

      if (i < j) {
         std::swap(data[i], data[j]);
      }
   }

   for (std::size_t span = 2; span <= length; span <<= 1) {
      const double angle = -2.0 * std::numbers::pi / static_cast<double> (span);
      const std::complex<double> rotation(std::cos(angle), std::sin(angle));
      const std::size_t half = span >> 1;

      for (std::size_t base = 0; base < length; base += span) {
         std::complex<double> factor(1.0, 0.0);

         for (std::size_t k = base; k < base + half; ++k) {
            const std::complex<double> even = data[k];
            const std::complex<double> odd  = data[k + half] * factor;

            data[k]        = even + odd;
            data[k + half] = even - odd;
            factor        *= rotation;
         }
      }
   }
}

// Прямое дискретное преобразование Фурье (эталон сверки для набора проверок)
// В рабочем тракте не применяется: сложность O(N²).
inline std::vector<std::complex<double> > dftDirect(const std::vector<std::complex<double> >& data) {
   const std::size_t length = data.size();
   std::vector<std::complex<double> > out(length);

   for (std::size_t k = 0; k < length; ++k) {
      std::complex<double> sum(0.0, 0.0);

      for (std::size_t n = 0; n < length; ++n) {
         const double angle = -2.0 * std::numbers::pi * static_cast<double> (k)
                              * static_cast<double> (n) / static_cast<double> (length);

         sum += data[n] * std::complex<double> (std::cos(angle), std::sin(angle));
      }
      out[k] = sum;
   }
   return out;
}
} // namespace glonass_service

#endif // SERVICE_FFT_RADIX2_H
