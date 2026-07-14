#include <algorithm>
#include <cassert>
#include <utility>
#include <vector>

#include "glonass/modulation.h"
#include "glonass/signal_combine.h"
#include "glonass/signal_source.h"

namespace glonass {
SignalSource::SignalSource(const SourceConfig& config) {
   assert(!config.letters.empty()       && "Блок Д (Д.4): |K|≥1 — пустая сетка не определена");
   assert(config.sampleRate       > 0   && "Fs > 0");
   assert(config.globalStartSample >= 0 && "n₀ ≥ 0");

   // Копия конфигураций литер, отсортированная по возрастанию k (Д.8 — детерминизм суммы).
   std::vector<LetterConfig> orderedLetters = config.letters;
   std::sort(orderedLetters.begin(), orderedLetters.end(),
             [](const LetterConfig& a, const LetterConfig& b) {
         return a.letter < b.letter;
      });

   // η = 1/√(Σ_{k∈K} A_k²) (Д.1/Д.4) — вычисляется однократно по набору амплитуд.
   std::vector<double> amplitudes;
   amplitudes.reserve(orderedLetters.size());

   for (const LetterConfig& letterConfig : orderedLetters) {
      assert(letterConfig.amplitude >= 0.0 && "A_k ≥ 0 (§ 0.1 поз.24)");
      amplitudes.push_back(letterConfig.amplitude);
   }
   normalizationFactor_ = glonass::normalizationFactor(amplitudes); // Д.1 (свободная функция блока Д)

   // init каждой литеры от n₀ (А.3/Б.4/В.4)
   letters_.reserve(orderedLetters.size());

   for (const LetterConfig& letterConfig : orderedLetters) {
      LetterState state;

      state.amplitude = letterConfig.amplitude;
      state.codePhase.init(config.globalStartSample, config.sampleRate,
                           letterConfig.codePhaseInit);                       // А.3
      state.navMessage.init(config.globalStartSample, config.sampleRate,
                            letterConfig.payloadOfLine);                      // Б.4
      state.carrierNco.init(config.globalStartSample, config.sampleRate,
                            carrierFreq(config.band, letterConfig.letter),    // f_k (В.2, [ИКД] 3.3.1.1)
                            config.referenceFreq, letterConfig.initialPhase); // f₀, φ_{0,k} (В.4)
      letters_.push_back(std::move(state));
   }
   letterSamples_.resize(letters_.size());                                    // буфер под |K| вкладов u_k[n]
   sampleIndex_ = config.globalStartSample;                                   // n = n₀ (r = 0)
}

OutputSample SignalSource::step() {
   // --- Фаза 1: съём выходов всех литер на n (состояния А/Б/В неизменны) → блок Д ---
   for (std::size_t i = 0; i < letters_.size(); ++i) {                                   // по ВОЗРАСТАНИЮ k (Д.8)
      LetterState& state                 = letters_[i];
      const int    chipIndex             = state.codePhase.chipIndex();                  // q_k (А.5)
      const Bit    codeBit               = rangingCode_.at(chipIndex);                   // c_k (А.6, общий код)
      const Bit    messageBit            = state.navMessage.messageBit();                // b_k (Б.5, съём ДО обновл.)
      const std::complex<double> carrier = state.carrierNco.carrier();                   // e_k (В.4, съём ДО обновл.)

      letterSamples_[i] = modulate(codeBit, messageBit, carrier, state.amplitude);       // u_k[n] (Г.3)
   }
   const OutputSample transmitterSample = combine(letterSamples_, normalizationFactor_); // Д.2/Д.9

   // --- Фаза 2: обновление состояний А/Б/В к n+1 (после съёма ВСЕХ литер — инвариант сетки) ---
   for (LetterState& state : letters_) {
      state.codePhase.step();  // P_{c,k}[n+1] (А.7)
      state.navMessage.step(); // фаза сообщения + событие границы строки j:199→0 (Б.8)
      state.carrierNco.step(); // Θ_k[n+1] (В.5)
   }
   ++sampleIndex_;             // n → n+1 (r → r+1)
   return transmitterSample;   // u[n] — отсчёт индекса n (Д.11)
}

SampleIndex SignalSource::sampleIndex() const {
   return sampleIndex_;
}

double SignalSource::normalizationFactor() const {
   return normalizationFactor_;
}

std::size_t SignalSource::letterCount() const {
   return letters_.size();
}
} // namespace glonass
