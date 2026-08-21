#ifndef SERVICE_STREAM_SESSION_H
#define SERVICE_STREAM_SESSION_H

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "glonass/signal_source_l1oc.h"
#include "glonass/source_config_l1oc.h"
#include "httplib.h"
#include "request_params_l1oc.h"

// Режим Б — потоковая выдача отсчётов (точка GET /v1/stream)
//
// Параметры фиксируются на прогон (Д_L1OC.4, п. 10.1): состав J, амплитуды и масштаб
// квантования вычисляются однократно при создании сессии и далее не меняются.
//
// Накопления записи нет ни на диске, ни в оперативной памяти: одновременно существует ровно
// один блок. Темп задаётся самым медленным звеном; обратное давление — штатное управление
// потоком TCP (заполнение окна приёма блокирует запись, а с нею и генерацию), своего протокола
// подтверждений нет.
namespace glonass_service {
enum class SampleFormat {
   cs16, // int16 I, int16 Q, чередование
   cf32  // float32 I, float32 Q, чередование
};

inline constexpr std::int64_t defaultBlockSamples = 65536;
inline constexpr std::int64_t minBlockSamples     = 1;
inline constexpr std::int64_t maxBlockSamples     = 1048576;                 // 8 МБ на блок в формате CF32

struct StreamRequest {
   std::int64_t        sampleRate    = glonass_params::defaultSampleRate;    // fs
   std::int64_t        referenceFreq = glonass_params::defaultReferenceFreq; // f0
   std::int64_t        startSample   = 0;                                    // n0 (§ 0.1 поз.25)
   std::vector<int>    satellites;                                           // J (поз.28)
   std::vector<double> amplitudes;                                           // A_j (поз.24)
   std::vector<double> initialPhases;                                        // φ_{0,j} (поз.46)

   std::int64_t sampleCount  = 0;
   std::int64_t blockSamples = defaultBlockSamples;
   SampleFormat format       = SampleFormat::cs16;
};

StreamRequest             parseStreamRequest(const httplib::Request& request);

// Конфигурация источника по параметрам запроса. Общая для режима Б и кадров:
glonass::SourceConfigL1OC sourceConfigOf(const StreamRequest& request);

std::size_t               bytesPerSample(SampleFormat format) noexcept;
const char*               formatName(SampleFormat format) noexcept;

double                    quantizationScaleCs16(const std::vector<double>& amplitudes) noexcept;

class StreamSession {
public:

   explicit StreamSession(const StreamRequest& request);

   // Очередной блок в байтах выходного формата; пустой промежуток — поток исчерпан.
   // Промежуток действителен до следующего вызова: буфер переиспользуется.
   std::span<const unsigned char> nextBlock();

   std::int64_t                   samplesEmitted() const noexcept {
      return samplesEmitted_;
   }

   // 0 — поток без предела
   std::int64_t sampleCount() const noexcept {
      return sampleCount_;
   }

   double normalizationFactor() const {
      return source_.normalizationFactor();
   }

   double quantizationScale() const noexcept { // k; для cf32 не применяется
      return quantizationScale_;
   }

private:

   glonass::SignalSourceL1OC source_;  // единственный экземпляр на весь поток
   std::vector<unsigned char> buffer_; // один блок, переиспользуется
   std::int64_t sampleCount_    = 0;
   std::int64_t blockSamples_   = defaultBlockSamples;
   std::int64_t samplesEmitted_ = 0;
   SampleFormat format_         = SampleFormat::cs16;
   double quantizationScale_    = 1.0;
};
} // namespace glonass_service

#endif // SERVICE_STREAM_SESSION_H
