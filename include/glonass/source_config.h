#ifndef SOURCE_CONFIG_H
#define SOURCE_CONFIG_H

#include <cstdint>
#include <vector>

#include "glonass/carrier_nco.h"
#include "glonass/nav_message.h"
#include "glonass/types.h"

namespace glonass {
// Конфигурация одной литеры k∈K для источника сигнала (, D3). amplitude = A_k применяется в Г
// (в letterSample_k) и в η (Д.2/Д.4); § 0.1 поз.24 (относительная амплитуда, не мощность).
struct LetterConfig {
   int             letter        = 0;   // k ∈ {−7,…,+6} (§ 0.2)
   double          amplitude     = 1.0; // A_k ≥ 0 (golden A_k=1)
   double          codePhaseInit = 0.0; // φ_{c0,k}, чипы (А.3, на литеру)
   double          initialPhase  = 0.0; // φ_{0,k}, рад (В.4, калибровочное смещение над n₀-фазой)
   PayloadProvider payloadOfLine;       // слой содержания (Б.11); пусто ⇒ вызывающий задаёт провайдер
};

// Конфигурация сетки FDMA на запуск. Множество K и амплитуды A_k фиксируются на запуск;
// смена отклоняется (Д.4 — сохранение фазовой непрерывности и однозначности нормировки).
struct SourceConfig {
   Band                      band;                  // диапазон несущей (В.2, [ИКД] 3.3.1.1)
   std::int64_t              sampleRate        = 0; // Fs, Гц (целое, > 0)
   std::int64_t              referenceFreq     = 0; // f₀, Гц (f_k одной литеры / f_центр сетки)
   SampleIndex               globalStartSample = 0; // n₀ ≥ 0 (единственный вход привязки, § 0.2)
   std::vector<LetterConfig> letters;               // K (|K|≥1; сортируется по возрастанию k, Д.8)
};
} // namespace glonass

#endif // SOURCE_CONFIG_H
