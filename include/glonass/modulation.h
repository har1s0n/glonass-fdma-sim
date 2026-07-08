#ifndef MODULATION_H
#define MODULATION_H

#include <complex>

#include "glonass/types.h"

namespace glonass {
// Модуляция ФМн/BPSK — комплексный вклад литеры u_k[n] in C (Г.1-Г.11). Блок КОМБИНАЦИОННЫЙ
// (Г.3/Г.6): состояния/инициализации/перехода НЕТ; выход на n — функция только текущих
// c_k[n], b_k[n], e_k[n]
//   mu_k = c_k XOR b_k (Г.1); g_k = 1-2*mu_k как ПЕРЕКЛЮЧАТЕЛЬ ЗНАКА фазора (Г.2/Г.8): mu=0 -> +e_k;
//   mu=1 -> -e_k (поворот фазы на pi), БЕЗ умножения на +-1. u_k = A_k*g_k*e_k (Г.3), координатно
//   I_k = A_k*g_k*cos(2pi*Theta_k/2^B) (Г.4), Q_k = A_k*g_k*sin(2pi*Theta_k/2^B) (Г.5).
// A_k (amplitude) — относительная амплитуда литеры (§0.1 поз.24, A_k>=0; golden A_k=1); применяется
//   ЗДЕСЬ. Общий масштаб сетки — Блок Д (§1), в u_k НЕ входит (исключение двойной нормировки).
// Воспроизводимость (§0.4): дискретная часть (mu_k, g_k) — ТОЧНО (XOR + смена знака в IEEE-754);
// вещественная (e_k, *A_k) — наследует уровень Блока В;
// Полярность mu=0 -> g=+1 (фаза 0), mu=1 -> g=-1 (фаза pi)
inline std::complex<double> modulate(Bit                  codeBit,    // c_k {0,1} (Блок А)
                                     Bit                  messageBit, // b_k {0,1} (Блок Б)
                                     std::complex<double> carrier,    // e_k in C  (Блок В)
                                     double               amplitude)  // A_k (нормировка — Блок Д)
noexcept {
   const Bit modulationBit = static_cast<Bit> (codeBit ^ messageBit); // mu_k = c_k XOR b_k (Г.1)

   // g_k = 1-2*mu_k как ПЕРЕКЛЮЧАТЕЛЬ ЗНАКА (Г.2/Г.8): mu=0 -> +e_k; mu=1 -> -e_k (фаза +pi). +-0,0:
   // при mu=1 знак нулевой компоненты инвертируется (-0,0)
   const double sigI = modulationBit ? -carrier.real() : carrier.real(); // Re(g_k*e_k)
   const double sigQ = modulationBit ? -carrier.imag() : carrier.imag(); // Im(g_k*e_k)

   const double letterI = amplitude * sigI;                              // I_k = A_k*g_k*cos(2pi*Theta_k/2^B) (Г.4) = Re u_k
   const double letterQ = amplitude * sigQ;                              // Q_k = A_k*g_k*sin(2pi*Theta_k/2^B) (Г.5) = Im u_k

   return { letterI, letterQ };                                          // letterSample = u_k[n] in C (Г.11); |u_k| = A_k
}
} // namespace glonass

#endif // MODULATION_H
