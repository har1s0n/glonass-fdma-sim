#ifndef IQ_CONVERT_H
#define IQ_CONVERT_H

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

// apps/iq_convert/convert.h — преобразование записи CF32 LE в CS16 LE для подачи во внешний
// программный приёмник. Инструмент сопряжения форматов
//
// Целевой формат CS16 — чередование int16 I,Q; знак Q НЕ инвертируется (соглашение SoapySDR
// I + j·Q). Байтовый порядок задаётся явно, как в glonass::IqSink, и от порядка платформы
// не зависит.
namespace glonass_tools {
constexpr std::int16_t cs16FullScale = 32767;

// Чтение float32 LE из буфера.
inline float readFloatLe(const unsigned char* bytes) noexcept {
   const std::uint32_t bits = static_cast<std::uint32_t> (bytes[0])
                              | (static_cast<std::uint32_t> (bytes[1]) << 8)
                              | (static_cast<std::uint32_t> (bytes[2]) << 16)
                              | (static_cast<std::uint32_t> (bytes[3]) << 24);
   float value = 0.0F;

   std::memcpy(&value, &bits, sizeof(value));
   return value;
}

// Запись int16 LE в буфер.
inline void writeInt16Le(unsigned char* bytes, std::int16_t value) noexcept {
   const std::uint16_t bits = static_cast<std::uint16_t> (value);

   bytes[0] = static_cast<unsigned char> (bits & 0xFFU);
   bytes[1] = static_cast<unsigned char> ((bits >> 8) & 0xFFU);
}

// Масштаб под полную шкалу формата: k = 32767 / max|x| по обеим координатам записи.
// Нулевая запись (max = 0) масштабу не поддаётся — принимается k = 1.
inline double scaleForPeakCs16(double peakAbs) noexcept {
   return peakAbs > 0.0 ? static_cast<double> (cs16FullScale) / peakAbs : 1.0;
}

// Квантование одной координаты: round(k·x) с округлением половины от нуля, ограничение ±32767.
// При k = scaleForPeakCs16(max|x|) ограничение по построению не срабатывает; оно оставлено
// на случай масштаба, заданного ключом --scale.
inline std::int16_t quantizeCs16(float value, double scale) noexcept {
   const long long q = std::llround(static_cast<double> (value) * scale);

   if (q > cs16FullScale) {
      return cs16FullScale;
   }

   if (q < -cs16FullScale) {
      return -cs16FullScale;
   }
   return static_cast<std::int16_t> (q);
}

// Максимум |x| по обеим координатам блока из sampleCount отсчётов CF32 LE (по 8 байт).
inline double peakAbsOfBlock(const unsigned char* in, std::size_t sampleCount) noexcept {
   double peak = 0.0;

   for (std::size_t i = 0; i < sampleCount * 2; ++i) { // координаты подряд: I,Q,I,Q,…
      const double magnitude = std::fabs(static_cast<double> (readFloatLe(in + i * 4)));

      if (magnitude > peak) {
         peak = magnitude;
      }
   }
   return peak;
}

// Блок из sampleCount отсчётов: CF32 LE (8 байт на отсчёт) -> CS16 LE (4 байта на отсчёт).
// Порядок координат I,Q сохраняется, знак Q не изменяется.
inline void convertBlockCs16(const unsigned char* in, std::size_t sampleCount, double scale,
                             unsigned char* out) noexcept {
   for (std::size_t i = 0; i < sampleCount * 2; ++i) {
      writeInt16Le(out + i * 2, quantizeCs16(readFloatLe(in + i * 4), scale));
   }
}
} // namespace glonass_tools

#endif // IQ_CONVERT_H
