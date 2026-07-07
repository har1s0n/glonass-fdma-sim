#pragma once
#include "glonass/types.h"
#include <cstdint>

namespace glonass {
// Аккумулятор кодовой фазы, на литеру (Ч3 А.3–А.7). Целочисленный, побитовая точность.
class CodePhase {
public:

   void init(SampleIndex  globalStartSample,    // n0 >= 0
             std::int64_t sampleRate,           // Fs
             double       codePhaseInit = 0.0); // phi_{c0,k} >= 0

   int           chipIndex() const;             // floor(accumulator / Fs) (А.5)
   void          step();                        // accumulator[n+1] (А.7)
   std::uint64_t accumulator() const;           // диагностика (под тесты)

private:

   std::uint64_t codePhaseAccumulator_ = 0; // P_{c,k}, uint64 (А.8)
   std::uint64_t codePhaseModulus_     = 0; // N*Fs, вычисляется в 64 битах (А.8)
   std::int64_t sampleRate_            = 0; // Fs
};
} // namespace glonass
