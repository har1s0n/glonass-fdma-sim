#include "state_metrics.h"

#include <cmath>
#include <limits>
#include <string>

#include "glonass/signal_combine.h"
#include "json_writer.h"
#include "service_version.h"

namespace glonass_service {
namespace {
constexpr const char* keySampleRate    = "fs";
constexpr const char* keyReferenceFreq = "f0";
constexpr const char* keyStartSample   = "n0";
constexpr const char* keySatellites    = "j";
constexpr const char* keyAmplitudes    = "amp";
constexpr const char* keyPhases        = "phi0";
constexpr const char* keyTime          = "t";

// Значение параметра либо пустой указатель, если параметр не задан
const std::string*paramOrNull(const httplib::Request& request, const char* key) {
   const auto found = request.params.find(key);

   return (found == request.params.end()) ? nullptr : &found->second;
}
} // namespace

const char*lineTypeName(glonass::LineTypeL1OC lineType) {
   switch (lineType) {
     case glonass::LineTypeL1OC::anomalous1: return "anomalous1";

     case glonass::LineTypeL1OC::anomalous2: return "anomalous2";

     default:                                return "normal";
   }
}

StateRequest parseStateRequest(const httplib::Request& request) {
   StateRequest parsed;
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

   if ((value = paramOrNull(request, keyTime)) != nullptr) {
      parsed.time = glonass_params::parseReal(*value, keyTime);
   }
   const std::string* satellites = paramOrNull(request, keySatellites);
   const std::string* amplitudes = paramOrNull(request, keyAmplitudes);
   const std::string* phases     = paramOrNull(request, keyPhases);

   glonass_params::requireSampleRate(parsed.sampleRate, keySampleRate);
   glonass_params::requireStartSample(parsed.startSample, keyStartSample);

   // Предусловие Б_L1OC.8 проверяется и в режиме А: координаты сообщения выводятся из Fs
   glonass_params::requireSymbolRate(parsed.sampleRate, keySampleRate);

   // Условие представимости В.2 здесь не проверяется — выводится полем representable
   parsed.satellites = glonass_params::parseSatellites(
      (satellites != nullptr) ? *satellites : glonass_params::defaultSatellites, keySatellites);
   parsed.amplitudes = glonass_params::parsePerSatellite(
      (amplitudes != nullptr) ? *amplitudes : glonass_params::defaultAmplitudes,
      parsed.satellites.size(), keyAmplitudes);
   parsed.initialPhases = glonass_params::parsePerSatellite(
      (phases != nullptr) ? *phases : glonass_params::defaultPhases,
      parsed.satellites.size(), keyPhases);

   // φ_{0,j} в показатели не входит, но проверяется на согласованность длины с |J|:
   // иначе ошибка конфигурации проявилась бы только при переходе к режиму Б
   glonass_params::requireAmplitudes(parsed.amplitudes, keyAmplitudes);
   return parsed;
}

StateMetrics computeStateMetrics(const StateRequest& request) {
   StateMetrics metrics;

   // n = n₀ + round(t·Fs); округление к ближайшему, половина — ОТ нуля (§ 0.1 поз.20).
   // Предел приведения задаётся разрядностью int64; NaN отсеивается тем же сравнением.
   const double offset = request.time * static_cast<double> (request.sampleRate);

   if (!(std::fabs(offset) < 9.0e18)) {
      throw glonass_params::ParamError(glonass_params::RejectKind::badValue, keyTime,
                                       "t: модельное время вне представимого диапазона");
   }
   metrics.sampleIndex = request.startSample + std::llround(offset);

   // Формулы Б_L1OC.4(4) выведены для n ≥ 0: на отрицательных операндах целочисленное
   // деление расходится с ⌊⌋ и координаты строки теряют смысл
   if (metrics.sampleIndex < 0) {
      throw glonass_params::ParamError(glonass_params::RejectKind::badValue, keyTime,
                                       "t: индекс отсчёта n = n₀ + round(t·Fs) отрицателен");
   }

   metrics.time                = request.time;
   metrics.satelliteCount      = request.satellites.size();
   metrics.normalizationFactor = glonass::normalizationFactor(request.amplitudes);       // Д_L1OC.1
   metrics.modelBandwidthHz    = glonass::modelBandwidthL1OC;                            // поз.34
   metrics.residualFreqHz      = glonass_params::residualFreq(request.referenceFreq);    // В.4
   metrics.representable       = glonass_params::isRepresentable(request.sampleRate,
                                                                 request.referenceFreq); // В.2

   // Координаты сообщения — подстановка текущего n в InitMessageAtSampleL1OC (Б_L1OC.9):
   // при регулярном потоке нормальных строк они зависят только от произведения n·R_с.
   // Слой содержания (Б_L1OC.11) выдаёт только нормальные строки, поэтому L_с постоянна.
   metrics.lineType   = glonass::LineTypeL1OC::normal;
   metrics.lineLength = 2 * glonass::lineBits(metrics.lineType); // L_с = 2·n_с (Б_L1OC.4(4))

   if (metrics.sampleIndex > std::numeric_limits<std::int64_t>::max() / glonass::symbolRateL1OC) {
      throw glonass_params::ParamError(glonass_params::RejectKind::badValue, keyTime,
                                       "t: произведение n·R_с вне разрядности счётчика символов");
   }
   const std::int64_t symbolCount = metrics.sampleIndex * glonass::symbolRateL1OC; // Z₀ = n·R_с
   const std::int64_t lineLength  = metrics.lineLength;

   metrics.lineIndex       = symbolCount / (lineLength * request.sampleRate);
   metrics.convSymbolIndex = static_cast<int> ((symbolCount / request.sampleRate) % lineLength);
   return metrics;
}

std::string stateMetricsJson(const StateMetrics& metrics) {
   JsonObject message;

   message.addInt("lineIndex", metrics.lineIndex);
   message.addString("lineType", lineTypeName(metrics.lineType));
   message.addInt("convSymbolIndex", metrics.convSymbolIndex);
   message.addInt("lineLength",      metrics.lineLength);

   JsonObject json;

   json.addInt("n", metrics.sampleIndex);
   json.addDouble("t", metrics.time);
   json.addString("band", band);
   json.addInt("satelliteCount", static_cast<std::int64_t> (metrics.satelliteCount));
   json.addDouble("normalizationFactor", metrics.normalizationFactor);
   json.addInt("modelBandwidthHz", metrics.modelBandwidthHz);
   json.addInt("residualFreqHz",   metrics.residualFreqHz);
   json.addBool("representable", metrics.representable);
   json.addRaw("message", message.str());
   return json.str();
}
} // namespace glonass_service
