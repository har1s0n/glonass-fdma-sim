#include "glonass/code_phase.h"
#include "glonass/numeric.h"
#include <cassert>
#include <cmath>

namespace glonass {
void CodePhase::init(SampleIndex globalStartSample, std::int64_t sampleRate, double codePhaseInit) {
   assert(sampleRate        > 0);
   assert(globalStartSample >= 0);  // граница v1: n0 >= 0
   assert(codePhaseInit    >= 0.0); // граница v1: phi_{c0,k} >= 0

   sampleRate_       = sampleRate;
   codePhaseModulus_ = static_cast<std::uint64_t> (codeLength)
                       * static_cast<std::uint64_t> (sampleRate); // N*Fs в 64 битах (А.8)

   // round(phi_{c0,k} * Fs) — half-away-from-zero, совпадает с «round» Ч3 (согл. п.2)
   const std::int64_t initTerm =
      std::llround(codePhaseInit * static_cast<double> (sampleRate)); // >= 0 в v1
   // член привязки: n0 * R_c mod (N*Fs) без переполнения (А.4(2))
   const std::uint64_t anchorTerm =
      mulMod(static_cast<std::uint64_t> (globalStartSample),
             static_cast<std::uint64_t> (codeRate),
             codePhaseModulus_);

   codePhaseAccumulator_ =
      (static_cast<std::uint64_t> (initTerm) % codePhaseModulus_ + anchorTerm)
      % codePhaseModulus_; // при phi=0, n0=0 -> 0
}

int CodePhase::chipIndex() const {
   // floor(accumulator / Fs) in [0, N) (А.5)
   return static_cast<int> (codePhaseAccumulator_ / static_cast<std::uint64_t> (sampleRate_));
}

void CodePhase::step() {
   // accumulator[n+1] = (accumulator + R_c) mod (N*Fs) (А.7)
   codePhaseAccumulator_ =
      (codePhaseAccumulator_ + static_cast<std::uint64_t> (codeRate)) % codePhaseModulus_;
}

std::uint64_t CodePhase::accumulator() const {
   return codePhaseAccumulator_;
}
} // namespace glonass
