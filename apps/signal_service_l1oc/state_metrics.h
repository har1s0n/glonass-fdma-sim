#ifndef SERVICE_STATE_METRICS_H
#define SERVICE_STATE_METRICS_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "glonass/nav_message_l1oc.h"
#include "httplib.h"
#include "request_params_l1oc.h"

// Режим А — показатели состояния
//
// Показатели вычисляются аналитически из конфигурации: ядро модели не
// запускается, отсчёты не формируются, состояние между обращениями не хранится. Коэффициент
// нормировки, число активных источников, полоса модели, остаточная расстройка, выполнение
// условия представимости, индекс и тип текущей строки навигационного сообщения
namespace glonass_service {
struct StateRequest {
   std::int64_t        sampleRate    = glonass_params::defaultSampleRate;    // fs
   std::int64_t        referenceFreq = glonass_params::defaultReferenceFreq; // f0
   std::int64_t        startSample   = 0;                                    // n0 (§ 0.1 поз.25)
   std::vector<int>    satellites;                                           // J (поз.28)
   std::vector<double> amplitudes;                                           // A_j (поз.24)
   std::vector<double> initialPhases;                                        // φ_{0,j} (поз.46)
   double              time = 0.0;                                           // t, с от привязки n₀
};

// Поля ответа; порядок членов задаёт порядок вывода JSON.
struct StateMetrics {
   std::int64_t          sampleIndex         = 0;                             // n = n₀ + round(t·Fs) (поз.20)
   double                time                = 0.0;                           // t
   std::size_t           satelliteCount      = 0;                             // |J|
   double                normalizationFactor = 1.0;                           // η (Д_L1OC.1)
   std::int64_t          modelBandwidthHz    = 0;                             // B_model (поз.34)
   std::int64_t          residualFreqHz      = 0;                             // Δf_j (В.4)
   bool                  representable       = false;                         // В.2
   std::int64_t          lineIndex           = 0;                             // № строки потока (Б_L1OC.9)
   glonass::LineTypeL1OC lineType            = glonass::LineTypeL1OC::normal; // тип строки (поз.37)
   int                   convSymbolIndex     = 0;                             // w_j[n] ∈ {0…L_с−1}
   int                   lineLength          = 0;                             // L_{с,j}
};

StateRequest parseStateRequest(const httplib::Request& request);
StateMetrics computeStateMetrics(const StateRequest& request);

std::string  stateMetricsJson(const StateMetrics& metrics);
const char*  lineTypeName(glonass::LineTypeL1OC lineType);
} // namespace glonass_service

#endif // SERVICE_STATE_METRICS_H
