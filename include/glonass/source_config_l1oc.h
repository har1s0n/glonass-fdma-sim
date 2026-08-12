#ifndef SOURCE_CONFIG_L1OC_H
#define SOURCE_CONFIG_L1OC_H

#include <cstdint>
#include <vector>

#include "glonass/nav_message_l1oc.h"
#include "glonass/types.h"

namespace glonass {
// Конфигурация одного источника j ∈ J тракта L1OC (§ 2_L1OC.3). Индекс источника — системный
// номер НКА j (§ 0.1 поз.28), роль литеры k FDMA-тракта; тракты не смешиваются (§ 2_L1OC.1).
struct SatelliteConfigL1OC {
   int                 satellite     = 1;   // j ∈ {1,…,63}; j = 0 резервный, в штатный состав не входит (поз.28; Д_L1OC.2)
   double              amplitude     = 1.0; // A_j ≥ 0 — относительная амплитуда (поз.24): в Г_L1OC.5, в η — Д_L1OC.1
   double              codePhaseInit = 0.0; // φ_{c0,j}, чипы уплотнения; 0 ≤ φ_{c0,j} < M (А_L1OC.5)
   double              initialPhase  = 0.0; // φ_{0,j}, рад (В.4); π/2 — квадратурное положение L1OC (поз.46)
   PayloadProviderL1OC payloadOfLineL1OC;
};

// Конфигурация активного множества J на запуск. J и амплитуды A_j фиксируются на запуск, смена
// отклоняется (Д_L1OC.4 — фазовая непрерывность и однозначность нормировки).
// Точка расширения на L2OC, L3OC
struct SourceConfigL1OC {
   std::int64_t                     sampleRate        = 0;               // Fs, Гц: Fs ≥ 4,092 МГц (В.2, поз.34), Fs ≥ R_с (Б_L1OC.8)
   std::int64_t                     referenceFreq     = carrierFreqL1OC; // f₀, Гц; при f₀ = f_L1OC ⇒ Δf_j = 0 (поз.26; § 1 (1.7))
   SampleIndex                      globalStartSample = 0;               // n₀ ≥ 0 — единственный вход привязки (поз.25)
   std::vector<SatelliteConfigL1OC> satellites;                          // J (|J| ≥ 1; сортируется по возрастанию j, Д_L1OC.8)
};
} // namespace glonass

#endif // SOURCE_CONFIG_L1OC_H
