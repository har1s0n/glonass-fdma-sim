#ifndef MODULATION_L1OC_H
#define MODULATION_L1OC_H

#include <complex>

#include "glonass/types.h"

namespace glonass {
// Блок Г_L1OC. Модуляция и почиповое временное уплотнение (Ч3 Г_L1OC.1-Г_L1OC.11)
// переменных состояния, инициализации и уравнения перехода
// НЕТ; выход на n — функция только текущих входов (Г_L1OC.2). Состояние сосредоточено в блоках-
// источниках: А_L1OC (кодовая фаза, из неё же sigma_j и мп_j), Б_L1OC (фаза символа СК, индекс
// символа и строка), В (фаза несущей).

// Символ уплотнённой последовательности П_L1OC,j[n] in {0,1} (Г_L1OC.1)-(Г_L1OC.3):
//   П_d = c_d XOR o XOR b (Г_L1OC.1) — трёхчленное сложение, [ИКД-L1OC] 2.1.2;
//   П_p = c_p XOR мп      (Г_L1OC.2) — наложение МП,          [ИКД-L1OC] 2.1.3;
//   sigma = 0 -> П = П_d;  sigma = 1 -> П = П_p (Г_L1OC.3).
// Уплотнение — ВЫБОР компоненты ([ИКД-L1OC] 2.1.4): на чиповом интервале передаётся символ ровно
// одной компоненты, компоненты НЕ суммируются. Раздельные амплитуды компонент не вводятся:
// равенство мощностей ([ИКД-L1OC] 2.1.1) обеспечено равным числом чиповых интервалов, отводимых
// компонентам уплотнителем (Г_L1OC.5). Порядок аргументов — по перечню входов Г_L1OC.2.
inline Bit multiplexL1OC(Bit codeBitD,                                                  // c_{d,j}[n] — символ ДК_L1OCd (Блок А_L1OC)
                         Bit codeBitP,                                                  // c_{p,j}[n] — символ ДК_L1OCp (Блок А_L1OC)
                         Bit meanderSymbol,                                             // мп_j[n] — символ МП          (Блок А_L1OC)
                         Bit componentSelect,                                           // sigma_j[n]: 0 — L1OCd, 1 — L1OCp (Блок А_L1OC)
                         Bit convSymbol,                                                // b_j[n] — символ СК           (Блок Б_L1OC)
                         Bit overlaySymbol)                                             // o_j[n] — символ ОК1          (Блок Б_L1OC)
noexcept {
   const Bit modulationBitD = static_cast<Bit> (codeBitD ^ overlaySymbol ^ convSymbol); // П_d (Г_L1OC.1)
   const Bit modulationBitP = static_cast<Bit> (codeBitP ^ meanderSymbol);              // П_p (Г_L1OC.2)

   return componentSelect ? modulationBitP : modulationBitD;                            // П_L1OC (Г_L1OC.3)
}

// Комплексный до-нормировочный вклад источника u_j[n] = A_j*g_j[n]*e_j[n] in C (Г_L1OC.5),
// координатно I_j = A_j*g_j*Re e_j (Г_L1OC.6), Q_j = A_j*g_j*Im e_j (Г_L1OC.7).
// g_j = 1-2*П_L1OC,j (Г_L1OC.4) применяется как ПЕРЕКЛЮЧАТЕЛЬ ЗНАКА фазора (Г_L1OC.8): П=0 -> +e_j;
// П=1 -> -e_j (поворот фазы на pi), БЕЗ умножения на +-1. +-0,0: при П=1 знак нулевой координаты
// инвертируется (-0,0).
inline std::complex<double> modulateL1OC(Bit                  multiplexedBit, // П_L1OC,j[n] in {0,1}
                                         std::complex<double> carrier,        // e_j[n] in C (Блок В)
                                         double               amplitude)      // A_j (нормировка — Д_L1OC)
noexcept {
   const double sigI = multiplexedBit ? -carrier.real() : carrier.real();     // Re(g_j*e_j)
   const double sigQ = multiplexedBit ? -carrier.imag() : carrier.imag();     // Im(g_j*e_j)

   const double sourceI = amplitude * sigI;                                   // I_j (Г_L1OC.6)
   const double sourceQ = amplitude * sigQ;                                   // Q_j (Г_L1OC.7)

   return { sourceI, sourceQ };                                               // sourceSample = u_j[n] (Г_L1OC.11); |u_j| = A_j
}
} // namespace glonass

#endif // MODULATION_L1OC_H
