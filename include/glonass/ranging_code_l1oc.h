#pragma once
#include "glonass/types.h"
#include <array>
#include <cstdint>

namespace glonass {
// Блок А_L1OC. Дальномерные коды L1OC (Ч3 А_L1OC.1–А_L1OC.11).
// Один экземпляр на НКА j: таблицы ДК обеих компонент и единый аккумулятор кодовой фазы.
// Отдельные аккумуляторы на компоненту не вводятся (А_L1OC.3): индекс чипа уплотнения
// m_j[n] задаёт и передаваемую компоненту, и индексы чипов обеих компонент.
class RangingCodeL1OC {
public:

   RangingCodeL1OC() = default;

   void initCodeTablesL1OC(int j);                                   // А_L1OC.4(1), псевдокод А_L1OC.9
   void initCodePhaseAtSampleL1OC(SampleIndex  globalStartSample,    // n0 >= 0
                                  std::int64_t sampleRate,           // Fs
                                  double       codePhaseInit = 0.0); // phi_{c0,j}, чипы; А_L1OC.5

   // --- фаза 1: съём выходов, состояние не изменяется (А_L1OC.7, А_L1OC.11) ---
   Bit                                 codeBitD() const;             // c_{d,j}[n] — символ ДК_L1OCd (А_L1OC.8)
   Bit                                 codeBitP() const;             // c_{p,j}[n] — символ ДК_L1OCp (А_L1OC.8)
   Bit                                 componentSelect() const;      // sigma_j[n]: 0 — L1OCd, 1 — L1OCp (А_L1OC.7)
   Bit                                 meanderSymbol() const;        // мп_j[n] — символ МП (А_L1OC.10)

   // диагностические выходы, для рабочего интерфейса необязательные (А_L1OC.11)
   int                                 multiplexChipIndex() const;   // m_j[n], 0 <= m_j <= 8183 (А_L1OC.6)
   int                                 chipIndexD() const;           // q_{d,j}[n], 0 <= q_d <= 1022 (А_L1OC.8)
   int                                 chipIndexP() const;           // q_{p,j}[n], 0 <= q_p <= 4091 (А_L1OC.8)

   // --- фаза 2: обновление состояния ---
   void                                step();                       // P_{c,j}[n+1] (А_L1OC.9)

   std::uint64_t                       codePhaseAccumulator() const;
   const std::array<Bit, codeLengthD> &codeTableD() const;
   const std::array<Bit, codeLengthP> &codeTableP() const;

private:

   std::array<Bit, codeLengthD> codeTableD_{};
   std::array<Bit, codeLengthP> codeTableP_{};
   std::uint64_t codePhaseAccumulator_ = 0;
   std::uint64_t codePhaseModulus_     = 0;
   std::int64_t sampleRate_            = 0;
};
} // namespace glonass
