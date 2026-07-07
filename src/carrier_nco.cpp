#include <cassert>
#include <cmath>

#include "glonass/carrier_nco.h"
#include "glonass/numeric.h"

namespace glonass {
namespace {
constexpr double kTwoPi    = 6.283185307179586476925286766559; // 2π
constexpr double kTwoPow32 = 4294967296.0;                     // 2^B (B=32) — полный оборот
} // namespace

std::int64_t carrierFreq(Band band, int letter) {
   const std::int64_t k = static_cast<std::int64_t> (letter);

   switch (band) { // [ИКД] 3.3.1.1
     case Band::L1OF: return 1'602'000'000LL + k * 562'500LL;
     case Band::L2OF: return 1'246'000'000LL + k * 437'500LL;
   }
   return 0;
}

void CarrierNco::init(SampleIndex  globalStartSample,
                      std::int64_t sampleRate,
                      std::int64_t carrierFreq,
                      std::int64_t referenceFreq,
                      double       initialPhase) {
   assert(globalStartSample >= 0);                                // n0 >= 0 (В.2)
   assert(sampleRate > 0);                                        // Fs > 0

   const std::int64_t residualFreq = carrierFreq - referenceFreq; // Δf_k (В.1), может <0

   // Условие представимости (В.9, поз.22): |Δf_k| + B_model <= Fs/2, B_model = codeRate.
   // Проверка на литеру эквивалентна max_{k in K} для всей сетки. Fs/2 в целых — консервативно.
   assert((residualFreq < 0 ? -residualFreq : residualFreq) + codeRate <= sampleRate / 2);

   const std::int64_t two_B = std::int64_t{ 1 } << phaseBits; // 2^B = 2^32

   // Δθ_k = RoundDivHalfAwayFromZero(Δf_k·2^B, Fs) mod_E 2^B (В.2). Числитель < 2^54 в int64 (В.8).
   const std::int64_t incr =
      euclideanModulo(roundDivHalfAwayFromZero(residualFreq * two_B, sampleRate), two_B);
   phaseIncrement_ = static_cast<std::uint32_t> (incr);

   // Θ_k[0] = (round(φ_{0,k}/(2π)·2^B) + MulMod(n0, Δθ_k, 2^B)) mod_E 2^B (В.3). При φ=0,n0=0 => 0.
   const std::int64_t phaseFromInitial =
      (initialPhase != 0.0)
            ? static_cast<std::int64_t> (std::llround(initialPhase / kTwoPi * static_cast<double> (two_B)))
            : 0;
   const std::uint64_t phaseFromStart =
      mulMod(static_cast<std::uint64_t> (globalStartSample),
             static_cast<std::uint64_t> (phaseIncrement_),
             static_cast<std::uint64_t> (two_B));
   carrierPhase_ = static_cast<std::uint32_t> (
      euclideanModulo(phaseFromInitial + static_cast<std::int64_t> (phaseFromStart), two_B));
}

std::complex<double> CarrierNco::carrier() const {
   // e_k[n] = cos(2π·Θ/2^B) + j·sin(2π·Θ/2^B) (В.4). Квадрантное свёртывание: старшие 2 бита Θ —
   // квадрант, младшие (B−2) — угол in [0,π/2). Осевые точки (Θ = q·2^(B−2)) выходят точно
   // (±1,0)/(0,±1) с −0,0 (§7(6)/(2)); внеосевые совпадают с прямым синтезом в пределах ε_NCO.
   // Таблицы нет, все B бит фазы используются => спуров фазового усечения нет (В.8).
   constexpr std::uint32_t quadrantShift = phaseBits - 2;                                        // B−2 = 30
   constexpr std::uint32_t quadrantMask  = (std::uint32_t{ 1 } << quadrantShift) - 1u;
   const std::uint32_t     quadrant      = carrierPhase_ >> quadrantShift;                       // 0..3
   const std::uint32_t     inPhase       = carrierPhase_ & quadrantMask;                         // фаза внутри квадранта
   const double angle                    = kTwoPi * (static_cast<double> (inPhase) / kTwoPow32); // in [0,π/2)
   const double c                        = std::cos(angle);
   const double s                        = std::sin(angle);

   switch (quadrant) {
     case 0:  return {  c,  s };
     case 1:  return { -s,  c };
     case 2:  return { -c, -s };
     default: return {  s, -c }; // quadrant == 3
   }
}

void CarrierNco::step() {
   carrierPhase_ += phaseIncrement_; // (Θ+Δθ) mod 2^B (В.5/В.8): естественный перенос за 2^32
}

std::uint32_t CarrierNco::phaseIncrement() const {
   return phaseIncrement_;
}

std::uint32_t CarrierNco::carrierPhase() const {
   return carrierPhase_;
}
} // namespace glonass
