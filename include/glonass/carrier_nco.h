#ifndef CARRIER_NCO_H
#define CARRIER_NCO_H

#include <complex>
#include <cstdint>

#include "glonass/types.h"

namespace glonass {
// Диапазон несущей (В.2, [ИКД] 3.3.1.1). Заглушка под CDMA — несущая.
enum class Band { L1OF, L2OF };

// f_k по литере/диапазону (В.2, [ИКД] 3.3.1.1):
//   L1OF: 1 602 000 000 + k·562 500;  L2OF: 1 246 000 000 + k·437 500;  k in −7…+6.
std::int64_t carrierFreq(Band band,
                         int  letter);

// Генератор фазора остаточной несущей e_k[n] in C, состояние — НА ЛИТЕРУ (Ч3 В.1–В.11).
// NCO с целочисленным аккумулятором фазы Θ_k (uint32, B=32); прямой синтез sin/cos от полной
// B-битной фазы (без таблицы, без усечения адреса)
class CarrierNco {
public:

   // Инициализация от n0 (В.4): Δf_k=f_k−f0; Δθ_k=RoundDivHalfAwayFromZero(Δf_k·2^B,Fs) mod_E 2^B;
   // Θ_k[0]=(round(φ_{0,k}/(2π)·2^B) + MulMod(n0,Δθ_k,2^B)) mod_E 2^B. При φ=0,n0=0 => Θ_k[0]=0.
   // Предусл. (В.9, поз.22): n0>=0; Fs>0; |Δf_k| + B_model <= Fs/2, B_model = codeRate.
   void init(SampleIndex  globalStartSample,    // n0 >= 0
             std::int64_t sampleRate,           // Fs, Гц
             std::int64_t carrierFreq,          // f_k, Гц ([ИКД] 3.3.1.1)
             std::int64_t referenceFreq,        // f0, Гц (f_k одной литеры / f_центр сетки)
             double       initialPhase = 0.0);  // φ_{0,k}, рад (калибровочное смещение над n0-фазой)

   std::complex<double> carrier() const;        // e_k[n]=cos+j·sin (В.4) — съём ДО обновления
   void                 step();                 // Θ_k[n+1]=(Θ_k[n]+Δθ_k) mod 2^B (В.5)

   // диагностика (тесты §7):
   std::uint32_t        phaseIncrement() const; // Δθ_k
   std::uint32_t        carrierPhase()   const; // Θ_k[n] in {0…2^B−1}

private:

   std::uint32_t carrierPhase_   = 0; // Θ_k[n] (В.3), беззнаковый B-бит
   std::uint32_t phaseIncrement_ = 0; // Δθ_k   (В.4), вычет [0,2^B)
};
} // namespace glonass

#endif // CARRIER_NCO_H
