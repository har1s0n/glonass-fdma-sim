#include <algorithm>
#include <cassert>
#include <vector>

#include "glonass/modulation_l1oc.h"
#include "glonass/signal_combine.h"
#include "glonass/signal_source_l1oc.h"

namespace glonass {
SignalSourceL1OC::SignalSourceL1OC(const SourceConfigL1OC& config) {
   assert(!config.satellites.empty()    && "Блок Д_L1OC (Д_L1OC.4): |J|≥1 — пустое множество не определено");
   assert(config.sampleRate        > 0  && "Fs > 0");
   assert(config.globalStartSample >= 0 && "n₀ ≥ 0");

   // Копия конфигураций НКА, отсортированная по возрастанию j (Д_L1OC.8 — детерминизм суммы).
   std::vector<SatelliteConfigL1OC> orderedSatellites = config.satellites;
   std::sort(orderedSatellites.begin(), orderedSatellites.end(),
             [](const SatelliteConfigL1OC& a, const SatelliteConfigL1OC& b) {
         return a.satellite < b.satellite;
      });

   // η = 1/√(Σ_{j∈J} A_j²) (Д_L1OC.1/Д_L1OC.4) — однократно по набору амплитуд.
   std::vector<double> amplitudes;
   amplitudes.reserve(orderedSatellites.size());

   for (const SatelliteConfigL1OC& satelliteConfig : orderedSatellites) {
      // j ∈ {1,…,63}: j = 0 резервный и в штатный состав не входит (поз.28; Д_L1OC.2). Квалификация
      // glonass:: обязательна — имя satelliteCount перекрыто одноимённым методом класса.
      assert(satelliteConfig.satellite >= 1
             && satelliteConfig.satellite < glonass::satelliteCount
             && "j ∈ {1,…,63}");
      assert(satelliteConfig.amplitude >= 0.0 && "A_j ≥ 0 (§ 0.1 поз.24)");
      amplitudes.push_back(satelliteConfig.amplitude);
   }
   assert(std::adjacent_find(orderedSatellites.begin(), orderedSatellites.end(),
                             [](const SatelliteConfigL1OC& a, const SatelliteConfigL1OC& b) {
         return a.satellite == b.satellite;
      }) == orderedSatellites.end() && "J — множество: повтор j недопустим");
   normalizationFactor_ = glonass::normalizationFactor(amplitudes); // Д_L1OC.1 (свободная функция блока Д)

   // init каждого НКА от n₀ (А_L1OC.4/А_L1OC.5, Б_L1OC.9, В.4)
   satellites_.reserve(orderedSatellites.size());

   for (const SatelliteConfigL1OC& satelliteConfig : orderedSatellites) {
      SatelliteState& state = satellites_.emplace_back();

      state.amplitude = satelliteConfig.amplitude;
      state.rangingCode.initCodeTablesL1OC(satelliteConfig.satellite);          // codeTableD[j], codeTableP[j] (А_L1OC.9)
      state.rangingCode.initCodePhaseAtSampleL1OC(config.globalStartSample,
                                                  config.sampleRate,
                                                  satelliteConfig.codePhaseInit); // P_{c,j}[0] (А_L1OC.5)
      state.navMessage.initMessageAtSampleL1OC(config.globalStartSample,
                                               config.sampleRate,
                                               satelliteConfig.payloadOfLineL1OC); // lineIndex/w/P_s[0] (Б_L1OC.9)
      state.carrierNco.init(config.globalStartSample, config.sampleRate,
                            carrierFreqL1OC,                                     // f_j = f_L1OC ([ИКД-L1OC] 2.1.1)
                            config.referenceFreq,                                // f₀ (при f₀ = f_L1OC ⇒ Δθ_j = 0)
                            satelliteConfig.initialPhase,                        // φ_{0,j} (В.4; поз.46)
                            modelBandwidthL1OC);                                 // B_model = 2·f_T1 (В.2, поз.34)
   }
   sourceSamples_.resize(satellites_.size());                                    // буфер под |J| вкладов u_j[n]
   sampleIndex_ = config.globalStartSample;                                      // n = n₀ (r = 0)
}

OutputSample SignalSourceL1OC::step() {
   // --- Фаза 1: съём выходов всех НКА на n (состояния А_L1OC/Б_L1OC/В неизменны) → блок Д_L1OC ---
   for (std::size_t i = 0; i < satellites_.size(); ++i) {                                 // по ВОЗРАСТАНИЮ j (Д_L1OC.8)
      SatelliteState& state = satellites_[i];

      // П_L1OC,j[n] (Г_L1OC.1)–(Г_L1OC.3): выходы А_L1OC и Б_L1OC снимаются ДО обновления
      const Bit multiplexedBit = multiplexL1OC(state.rangingCode.codeBitD(),         // c_{d,j} (А_L1OC.8)
                                               state.rangingCode.codeBitP(),         // c_{p,j} (А_L1OC.8)
                                               state.rangingCode.meanderSymbol(),    // мп_j    (А_L1OC.10)
                                               state.rangingCode.componentSelect(),  // σ_j     (А_L1OC.7)
                                               state.navMessage.convSymbol(),        // b_j     (Б_L1OC.8)
                                               state.navMessage.overlaySymbol());    // o_j     (Б_L1OC.9)

      sourceSamples_[i] = modulateL1OC(multiplexedBit,
                                       state.carrierNco.carrier(),                   // e_j     (В.4)
                                       state.amplitude);                             // u_j[n]  (Г_L1OC.5)
   }
   const OutputSample transmitterSample = combine(sourceSamples_, normalizationFactor_); // Д_L1OC.2/Д_L1OC.9

   // --- Фаза 2: обновление состояний к n+1 (после съёма ВСЕХ источников — инвариант сетки) ---
   for (SatelliteState& state : satellites_) {
      state.rangingCode.step(); // P_{c,j}[n+1] (А_L1OC.9)
      state.navMessage.step();  // фаза символа СК + событие границы строки w: L_с−1→0 (Б_L1OC.10)
      state.carrierNco.step();  // Θ_j[n+1] (В.5)
   }
   ++sampleIndex_;              // n → n+1 (r → r+1)
   return transmitterSample;    // u[n] — отсчёт индекса n (Д_L1OC.11)
}

SampleIndex SignalSourceL1OC::sampleIndex() const {
   return sampleIndex_;
}

double SignalSourceL1OC::normalizationFactor() const {
   return normalizationFactor_;
}

std::size_t SignalSourceL1OC::satelliteCount() const {
   return satellites_.size();
}
} // namespace glonass
