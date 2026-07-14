#ifndef IQ_SINK_H
#define IQ_SINK_H

#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <ios>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

// Экспорт I/Q сетки FDMA в бинарный файл и сопроводительные метаданные.
// Этап 6 (модуль запуска / экспорт); в Ч3 псевдокода нет — стиль проекта (camelCase/PascalCase),
// D10 дословно не диктует. Header-only, как блоки Г/Д. Зависимостей от заголовков ядра нет
// (принимает std::complex<float>); OutputSample ядра = std::complex<float> ⇒ подаётся напрямую.
namespace glonass {

// ───────────────────────────── Писатель CF32 ─────────────────────────────
// Поток отсчётов u[n] → сырой бинарный файл: чередование I,Q,I,Q,… по float32 на компоненту
// (CF32 LE interleaved). Это нативный gr_complex GNSS-SDR (item_type=gr_complex) и fc32 UHD/Soapy:
// файл читается File_Signal_Source напрямую. Заголовка нет (сырой поток) ⇒ размер файла = N·8 байт.
// Байтовый порядок little-endian задаётся ЯВНО (побайтно) — не зависит от порядка платформы;
// на LE-хостах путь эквивалентен прямой записи. Запись потоковая: вся выборка в ОЗУ не держится.
class IqSink {
public:
   explicit IqSink(const std::string& path)
      : stream_(path, std::ios::binary | std::ios::out | std::ios::trunc) {
      if (!stream_) {
         throw std::runtime_error("IqSink: не удалось открыть для записи: " + path);
      }
   }

   // Один комплексный отсчёт → 8 байт: float32 I (LE), затем float32 Q (LE). Порядок I,Q — Ч3 Д.11.
   void writeSample(std::complex<float> sample) {
      writeFloatLe(sample.real());   // I[n] = Re u[n]
      writeFloatLe(sample.imag());   // Q[n] = Im u[n]
      ++samplesWritten_;
   }

   std::size_t samplesWritten() const noexcept { return samplesWritten_; }

   // Явное закрытие с проверкой потока (иначе — в деструкторе, но без сигнала об ошибке).
   void close() {
      stream_.flush();
      if (!stream_) {
         throw std::runtime_error("IqSink: ошибка записи потока I/Q");
      }
      stream_.close();
   }

private:
   static std::uint32_t bitsOf(float value) noexcept {
      static_assert(sizeof(float) == sizeof(std::uint32_t), "требуется 32-битный float");
      std::uint32_t bits = 0;
      std::memcpy(&bits, &value, sizeof(bits));   // корректный type-pun (не UB, в отличие от reinterpret)
      return bits;
   }

   void writeFloatLe(float value) {
      const std::uint32_t    bits     = bitsOf(value);
      const unsigned char    bytes[4] = {                       // младший байт первым (LE)
         static_cast<unsigned char>( bits        & 0xFFu),
         static_cast<unsigned char>((bits >>  8) & 0xFFu),
         static_cast<unsigned char>((bits >> 16) & 0xFFu),
         static_cast<unsigned char>((bits >> 24) & 0xFFu),
      };
      stream_.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
   }

   std::ofstream stream_;
   std::size_t   samplesWritten_ = 0;
};

// ──────────────────────────── Sidecar (метаданные) ────────────────────────────
// Рядом с бинарником — плоский JSON: без него сырой поток неоднозначен для стороны НАП/В&В.
// Ориентирован на программный НАП (GNSS-SDR): item_type, частота дискретизации, центр РЧ (LO,
// = f₀ ядра), диапазон, состав литер. Пик/PAPR-backoff не включаются (целочисленного ЦАП нет —
// потребитель софтовый, float).
struct IqMetadata {
   std::string      generator    = "glonass_fdma_core";
   std::string      format       = "cf32";        // complex float32
   std::string      itemType     = "gr_complex";  // GNSS-SDR File_Signal_Source
   std::string      byteOrder    = "little-endian";
   std::string      layout       = "interleaved-iq";
   int              bytesPerSample = 8;
   std::int64_t     sampleRateHz = 0;             // Fs
   std::int64_t     rfCenterHz   = 0;             // f₀ (центр набора / несущая одной литеры) = LO НАП
   std::string      band;                         // "L1" | "L2"
   std::int64_t     n0           = 0;             // globalStartSample
   std::size_t      numSamples   = 0;             // фактически записано
   std::vector<int> literals;                     // K, по возрастанию
   std::vector<double> amplitudes;                // A_k, в порядке literals
   double           eta          = 1.0;           // η = 1/√(Σ A_k²)
   std::string      payload      = "zero";        // v1
};

inline void writeSidecarJson(const std::string& path, const IqMetadata& meta) {
   std::ofstream out(path, std::ios::out | std::ios::trunc);
   if (!out) {
      throw std::runtime_error("writeSidecarJson: не удалось открыть: " + path);
   }
   out.setf(std::ios::fmtflags(0), std::ios::floatfield);   // не fixed/scientific принудительно

   auto intArray = [](const std::vector<int>& v) {
      std::string s = "[";
      for (std::size_t i = 0; i < v.size(); ++i) {
         if (i) s += ", ";
         s += std::to_string(v[i]);
      }
      return s + "]";
   };
   auto dblArray = [](const std::vector<double>& v) {
      std::string s = "[";
      char buf[32];
      for (std::size_t i = 0; i < v.size(); ++i) {
         if (i) s += ", ";
         std::snprintf(buf, sizeof(buf), "%.17g", v[i]);
         s += buf;
      }
      return s + "]";
   };
   char etaBuf[32];
   std::snprintf(etaBuf, sizeof(etaBuf), "%.17g", meta.eta);

   out << "{\n"
       << "  \"generator\": \""       << meta.generator      << "\",\n"
       << "  \"format\": \""          << meta.format         << "\",\n"
       << "  \"gnss_sdr_item_type\": \"" << meta.itemType    << "\",\n"
       << "  \"byte_order\": \""      << meta.byteOrder      << "\",\n"
       << "  \"layout\": \""          << meta.layout         << "\",\n"
       << "  \"bytes_per_sample\": "  << meta.bytesPerSample  << ",\n"
       << "  \"sample_rate_hz\": "    << meta.sampleRateHz    << ",\n"
       << "  \"rf_center_hz\": "      << meta.rfCenterHz      << ",\n"
       << "  \"band\": \""            << meta.band           << "\",\n"
       << "  \"n0\": "                << meta.n0              << ",\n"
       << "  \"num_samples\": "       << meta.numSamples      << ",\n"
       << "  \"literals\": "          << intArray(meta.literals)    << ",\n"
       << "  \"amplitudes\": "        << dblArray(meta.amplitudes)  << ",\n"
       << "  \"eta\": "               << etaBuf               << ",\n"
       << "  \"payload\": \""         << meta.payload        << "\"\n"
       << "}\n";
   if (!out) {
      throw std::runtime_error("writeSidecarJson: ошибка записи: " + path);
   }
}

} // namespace glonass

#endif // IQ_SINK_H
