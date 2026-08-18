#include "stream_session.h"

#include <cmath>
#include <cstring>
#include <string>
#include <utility>

#include "convert.h"
#include "glonass/signal_combine.h"
#include "glonass/source_config_l1oc.h"
#include "glonass/types.h"

namespace glonass_service {
namespace {
constexpr const char* keySampleRate    = "fs";
constexpr const char* keyReferenceFreq = "f0";
constexpr const char* keyStartSample   = "n0";
constexpr const char* keySatellites    = "j";
constexpr const char* keyAmplitudes    = "amp";
constexpr const char* keyPhases        = "phi0";
constexpr const char* keySampleCount   = "n";
constexpr const char* keySeconds       = "seconds";
constexpr const char* keyFormat        = "format";
constexpr const char* keyBlockSamples  = "blockSamples";

constexpr std::size_t bytesPerCf32 = 8; // float32 I + float32 Q
constexpr std::size_t bytesPerCs16 = 4; // int16 I + int16 Q

// Значение параметра либо пустой указатель, если параметр не задан.
const std::string*paramOrNull(const httplib::Request& request, const char* key) {
   const auto found = request.params.find(key);

   return (found == request.params.end()) ? nullptr : &found->second;
}

void writeFloatLe(unsigned char* bytes, float value) noexcept {
   static_assert(sizeof(float) == sizeof(std::uint32_t), "требуется 32-битный float");
   std::uint32_t bits = 0;

   std::memcpy(&bits, &value, sizeof(bits)); // корректный type-pun (не UB, в отличие от reinterpret)
   bytes[0] = static_cast<unsigned char> (bits         & 0xFFU);
   bytes[1] = static_cast<unsigned char> ((bits >>  8) & 0xFFU);
   bytes[2] = static_cast<unsigned char> ((bits >> 16) & 0xFFU);
   bytes[3] = static_cast<unsigned char> ((bits >> 24) & 0xFFU);
}

// Нулевая ЦИ, строка нормального типа (Д_L1OC.10) — слой содержания Б_L1OC.11 в область
// текущих работ не входит, как и в модуле запуска glonass_signal_gen_l1oc.
glonass::PayloadProviderL1OC zeroPayloadProviderL1OC() {
   return [](std::int64_t /*lineIndex*/) {
             return glonass::LineContentL1OC{}; // lineType = normal, ЦИ = 0 (value-init)
   };
}

// Длительность по seconds: n = round(seconds·Fs), половина — ОТ нуля (§ 0.1 поз.20), как в
// модуле запуска. Предел приведения задаётся разрядностью int64; NaN отсеивается сравнением.
std::int64_t sampleCountFromSeconds(double seconds, std::int64_t sampleRate) {
   const double offset = seconds * static_cast<double> (sampleRate);

   if (!(std::fabs(offset) < 9.0e18)) {
      throw glonass_params::ParamError(glonass_params::RejectKind::badValue, keySeconds,
                                       "seconds: длительность вне представимого диапазона");
   }
   return std::llround(offset);
}

glonass::SourceConfigL1OC sourceConfigOf(const StreamRequest& request) {
   glonass::SourceConfigL1OC config;

   config.sampleRate        = request.sampleRate;
   config.referenceFreq     = request.referenceFreq;
   config.globalStartSample = static_cast<glonass::SampleIndex> (request.startSample);

   // Порядок по возрастанию j (Д_L1OC.8) устанавливает сам источник; здесь сохраняется
   // соответствие satellites ↔ amplitudes ↔ initialPhases порядка разбора.
   for (std::size_t index = 0; index < request.satellites.size(); ++index) {
      glonass::SatelliteConfigL1OC satelliteConfig;

      satelliteConfig.satellite         = request.satellites[index];
      satelliteConfig.amplitude         = request.amplitudes[index];
      satelliteConfig.codePhaseInit     = 0.0; // φ_{c0,j} = 0 (§ 0.1 поз.47)
      satelliteConfig.initialPhase      = request.initialPhases[index];
      satelliteConfig.payloadOfLineL1OC = zeroPayloadProviderL1OC();
      config.satellites.push_back(std::move(satelliteConfig));
   }
   return config;
}
} // namespace

std::size_t bytesPerSample(SampleFormat format) noexcept {
   return (format == SampleFormat::cf32) ? bytesPerCf32 : bytesPerCs16;
}

const char*formatName(SampleFormat format) noexcept {
   return (format == SampleFormat::cf32) ? "cf32" : "cs16";
}

double quantizationScaleCs16(const std::vector<double>& amplitudes) noexcept {
   double sumAmplitudes = 0.0;

   for (const double amplitude : amplitudes) { // A_j ≥ 0 (поз.24) ⇒ Σ A_j = Σ |A_j|
      sumAmplitudes += amplitude;
   }

   // η из блока Д (Д_L1OC.1) — та же функция, что применяет ядро: масштаб не должен зависеть
   // от отдельного пересчёта нормировки.
   const double bound = glonass::normalizationFactor(amplitudes) * sumAmplitudes;

   return static_cast<double> (glonass_tools::cs16FullScale) / bound;
}

StreamRequest parseStreamRequest(const httplib::Request& request) {
   StreamRequest parsed;
   const std::string* value = nullptr;

   if ((value = paramOrNull(request, keySampleRate)) != nullptr) {
      parsed.sampleRate = glonass_params::parseInteger(*value, keySampleRate);
   }

   if ((value = paramOrNull(request, keyReferenceFreq)) != nullptr) {
      parsed.referenceFreq = glonass_params::parseInteger(*value, keyReferenceFreq);
   }

   if ((value = paramOrNull(request, keyStartSample)) != nullptr) {
      parsed.startSample = glonass_params::parseInteger(*value, keyStartSample);
   }

   glonass_params::requireSampleRate(parsed.sampleRate, keySampleRate);
   glonass_params::requireStartSample(parsed.startSample, keyStartSample);

   // Точка выполняет прогон, поэтому нереализуемая конфигурация отклоняется до выдачи тела
   // (В.2, поз.34; Б_L1OC.8) — как в модуле запуска.
   glonass_params::requireRepresentable(parsed.sampleRate, parsed.referenceFreq, keySampleRate);
   glonass_params::requireSymbolRate(parsed.sampleRate, keySampleRate);

   // Длительность: n старше seconds; не задано ни то, ни другое — поток без предела (§ 5.2)
   const std::string* sampleCount = paramOrNull(request, keySampleCount);
   const std::string* seconds     = paramOrNull(request, keySeconds);

   if (sampleCount != nullptr) {
      parsed.sampleCount = glonass_params::parseInteger(*sampleCount, keySampleCount);

      if (parsed.sampleCount <= 0) {
         throw glonass_params::ParamError(glonass_params::RejectKind::badValue, keySampleCount,
                                          "n: число отсчётов должно быть > 0");
      }
   } else if (seconds != nullptr) {
      parsed.sampleCount = sampleCountFromSeconds(
         glonass_params::parseReal(*seconds, keySeconds), parsed.sampleRate);

      if (parsed.sampleCount <= 0) {
         throw glonass_params::ParamError(glonass_params::RejectKind::badValue, keySeconds,
                                          "seconds: длительность должна давать ≥ 1 отсчёт");
      }
   }

   if ((value = paramOrNull(request, keyBlockSamples)) != nullptr) {
      parsed.blockSamples = glonass_params::parseInteger(*value, keyBlockSamples);

      if ((parsed.blockSamples < minBlockSamples) || (parsed.blockSamples > maxBlockSamples)) {
         throw glonass_params::ParamError(glonass_params::RejectKind::badValue, keyBlockSamples,
                                          "blockSamples: вне диапазона "
                                          + std::to_string(minBlockSamples) + "…"
                                          + std::to_string(maxBlockSamples));
      }
   }

   if ((value = paramOrNull(request, keyFormat)) != nullptr) {
      if (*value == "cs16") {
         parsed.format = SampleFormat::cs16;
      } else if (*value == "cf32") {
         parsed.format = SampleFormat::cf32;
      } else {
         throw glonass_params::ParamError(glonass_params::RejectKind::badValue, keyFormat,
                                          "format: допустимы cf32 и cs16: " + *value);
      }
   }
   const std::string* satellites = paramOrNull(request, keySatellites);
   const std::string* amplitudes = paramOrNull(request, keyAmplitudes);
   const std::string* phases     = paramOrNull(request, keyPhases);

   parsed.satellites = glonass_params::parseSatellites(
      (satellites != nullptr) ? *satellites : glonass_params::defaultSatellites, keySatellites);
   parsed.amplitudes = glonass_params::parsePerSatellite(
      (amplitudes != nullptr) ? *amplitudes : glonass_params::defaultAmplitudes,
      parsed.satellites.size(), keyAmplitudes);
   parsed.initialPhases = glonass_params::parsePerSatellite(
      (phases != nullptr) ? *phases : glonass_params::defaultPhases,
      parsed.satellites.size(), keyPhases);

   // A_j ≥ 0 (поз.24) и Σ A_j² > 0 (предусловие Д_L1OC.1): ядро опирается на assert,
   // отключаемый в сборке Release.
   glonass_params::requireAmplitudes(parsed.amplitudes, keyAmplitudes);
   return parsed;
}

StreamSession::StreamSession(const StreamRequest& request)
   : source_(sourceConfigOf(request)),
   sampleCount_(request.sampleCount),
   blockSamples_(request.blockSamples),
   format_(request.format),
   quantizationScale_(quantizationScaleCs16(request.amplitudes)) {
   // Буфер на один блок: в конвейере одновременно находится один блок, накопления нет.
   buffer_.resize(static_cast<std::size_t> (blockSamples_) * bytesPerSample(format_));
}

std::span<const unsigned char> StreamSession::nextBlock() {
   // Поток без предела (sampleCount_ = 0) выдаёт полные блоки неограниченно.
   const std::int64_t remaining = (sampleCount_ > 0) ? (sampleCount_ - samplesEmitted_)
                                                     : blockSamples_;
   const std::int64_t count = (remaining < blockSamples_) ? remaining : blockSamples_;

   if (count <= 0) {
      return {};
   }
   const std::size_t width = bytesPerSample(format_);
   unsigned char*    out   = buffer_.data();

   for (std::int64_t index = 0; index < count; ++index) {
      // Фаза1(съём ВСЕХ НКА на n → Д_L1OC)→(I,Q); Фаза2(→ n+1) — § 2_L1OC.3
      const glonass::OutputSample sample = source_.step();

      if (format_ == SampleFormat::cf32) {
         writeFloatLe(out,     sample.real()); // I[n] = Re u[n] (Д_L1OC.11)
         writeFloatLe(out + 4, sample.imag()); // Q[n] = Im u[n]
      } else {
         glonass_tools::writeInt16Le(out,
                                     glonass_tools::quantizeCs16(sample.real(), quantizationScale_));
         glonass_tools::writeInt16Le(out + 2,
                                     glonass_tools::quantizeCs16(sample.imag(), quantizationScale_));
      }
      out += width;
   }
   samplesEmitted_ += count;
   return { buffer_.data(), static_cast<std::size_t> (count) * width };
}
} // namespace glonass_service
