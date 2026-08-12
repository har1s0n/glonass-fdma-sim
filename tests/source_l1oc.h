#ifndef SOURCE_L1OC_H
#define SOURCE_L1OC_H

// tests/source_l1oc.h — сцепка блоков А_L1OC + Б_L1OC -> Г_L1OC для одного НКА, общая для
// Порядок вычислений — § 2_L1OC.3: фаза 1 (съём выходов, состояния
// не изменяются), фаза 2 (продвижение состояний к n+1)
#include "glonass/modulation_l1oc.h"
#include "glonass/nav_message_l1oc.h"
#include "glonass/ranging_code_l1oc.h"
#include "glonass/types.h"

#include <complex>
#include <cstdint>

namespace testutil {
constexpr std::int64_t sampleRateL1OC = 20000000;   // Fs = 20,0 МГц
constexpr int samplesPer8ms           = 160000;     // период замыкания кодовой фазы (Г_L1OC.10, Д_L1OC.10)

const std::complex<double> kUnitPhasor{ 1.0, 0.0 }; // e_j при f0 = f_L1OC, phi_0 = 0 (§ 1, (1.7))

// Режим контрольных примеров Г_L1OC.10 и Д_L1OC.10: ЦИ нулевая, строка нормального типа, S_j = 0.
inline glonass::PayloadProviderL1OC zeroProvider() {
   return [](std::int64_t) {
             return glonass::LineContentL1OC{};
   };
}

// Сцепка А_L1OC + Б_L1OC -> Г_L1OC для одного НКА (§ 2_L1OC.3): фаза 1 — съём выходов
// блоков-источников и вычисление П_L1OC, фаза 2 — продвижение состояний к n+1.
class SourceL1OC {
public:

   explicit SourceL1OC(int j, glonass::SampleIndex globalStartSample = 0) {
      code_.initCodeTablesL1OC(j);
      code_.initCodePhaseAtSampleL1OC(globalStartSample, sampleRateL1OC, 0.0);
      message_.initMessageAtSampleL1OC(globalStartSample, sampleRateL1OC, zeroProvider());
   }

   // --- фаза 1 ---
   glonass::Bit multiplexedBit() const { // П_L1OC,j[n] (Г_L1OC.1)-(Г_L1OC.3)
      return glonass::multiplexL1OC(code_.codeBitD(), code_.codeBitP(), code_.meanderSymbol(),
                                    code_.componentSelect(), message_.convSymbol(),
                                    message_.overlaySymbol());
   }

   std::complex<double> sourceSample(std::complex<double> carrier   = kUnitPhasor,
                                     double               amplitude = 1.0) const {
      return glonass::modulateL1OC(multiplexedBit(), carrier, amplitude); // u_j[n] (Г_L1OC.5)
   }

   // --- фаза 2 ---
   void step() {
      code_.step();
      message_.step();
   }

   const glonass::RangingCodeL1OC &code() const {
      return code_;
   }

   const glonass::NavMessageL1OC &message() const {
      return message_;
   }

private:

   glonass::RangingCodeL1OC code_;
   glonass::NavMessageL1OC message_;
};
} // namespace testutil

#endif // SOURCE_L1OC_H
