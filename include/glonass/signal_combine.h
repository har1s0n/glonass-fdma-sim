#ifndef SIGNAL_COMBINE_H
#define SIGNAL_COMBINE_H

#include <cmath>
#include <complex>
#include <span>

#include "glonass/types.h"

namespace glonass {
// Блок Д — суммарный сигнал и нормировка (Д.1-Д.11). Блок КОМБИНАЦИОННЫЙ (Д.3/Д.6):
// переменных состояния/инициализации/перехода НЕТ; выход на n — функция только текущих letterSample_k[n] (из Г)
// и предвычисленного η. η — не состояние, а константа сетки (Д.3).

// η = 1/√(Σ_{k∈K} A_k²) (Д.1/Д.4). Нормировка по СРЕДНЕЙ МОЩНОСТИ (Ч2 § 11.2): η²·Σ A_k² = 1
// ⇒ номинальная мощность P_nom = E{|u[n]|²} = 1 (Д.8). Вычисляется ОДНОКРАТНО при фиксации
// множества K и амплитуд A_k на запуск (в step источника сигнала НЕ пересчитывается). При A_k=1 ∀k ⇒
// η=1/√|K|; при |K|=1 ⇒ η=1; пример A={1,2} ⇒ η=1/√5 (Д.4)
inline double normalizationFactor(std::span<const double> amplitudes) noexcept {
   double sumSquares = 0.0;

   for (double amplitude : amplitudes) {   // порядок несуществен (Σ A_k² ≥ 0)
      sumSquares += amplitude * amplitude; // A_k² (A_k ≥ 0, § 0.1 поз.24)
   }
   return 1.0 / std::sqrt(sumSquares);     // Д.1 (предусл. Σ A_k² > 0)
}

// transmitterSample = η · Σ_{k∈K} letterSample_k (Д.2/Д.5/Д.9). Раздельно по координатам:
// sumI = Σ Re u_k, sumQ = Σ Im u_k (Д.8/Д.9). Порядок суммы задаётся порядком элементов span —
// вызывающий подаёт по ВОЗРАСТАНИЮ k (Д.8: сложение float неассоциативно ⇒ фиксированный
// порядок обеспечивает бит-воспроизводимость). Домножение на η — ПОСЛЕ суммы (Д.8; A_k уже в
// letterSample_k из Г ⇒ исключение двойной нормировки). Приведение double→float32 — в самом
// конце, единичное округление на компоненту (round-to-nearest-even, Д.9)
// Возврат — эталонный отсчёт сетки: outputI = Re u[n], outputQ = Im u[n] (Д.11).
inline OutputSample combine(std::span<const std::complex<double> > letterSamples,
                            double                                 normalizationFactor) noexcept {
   double sumI = 0.0;                                               // Σ Re u_k (Д.9)
   double sumQ = 0.0;                                               // Σ Im u_k (Д.9)

   for (const std::complex<double>& letterSample : letterSamples) { // по ВОЗРАСТАНИЮ k (Д.7/Д.8)
      sumI += letterSample.real();
      sumQ += letterSample.imag();
   }
   const double outputI = normalizationFactor * sumI; // Re u[n] = η·Σ Re u_k (Д.3/Д.5)
   const double outputQ = normalizationFactor * sumQ; // Im u[n] = η·Σ Im u_k (Д.3/Д.5)

   return { static_cast<float> (outputI),             // double→float32 (Д.9); outputI=Re (Д.11)
            static_cast<float> (outputQ) };           // double→float32 (Д.9); outputQ=Im (Д.11)
}
} // namespace glonass

#endif // SIGNAL_COMBINE_H
