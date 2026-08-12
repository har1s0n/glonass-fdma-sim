#ifndef SIGNAL_SOURCE_L1OC_H
#define SIGNAL_SOURCE_L1OC_H

#include <complex>
#include <cstddef>
#include <vector>

#include "glonass/carrier_nco.h"
#include "glonass/nav_message_l1oc.h"
#include "glonass/ranging_code_l1oc.h"
#include "glonass/source_config_l1oc.h"
#include "glonass/types.h"

namespace glonass {
// Источник суммарного сигнала тракта L1OC. Сборка А_L1OC+Б_L1OC+В+Г_L1OC+Д_L1OC: на НКА j∈J
// В отличие от FDMA-тракта таблицы ДК ведутся НА ИСТОЧНИК (НС2 = j, поз.29/30), общего кода нет.
// Двухфазный поотсчётный шаг (§ 2_L1OC.2, § 2_L1OC.3):
//   Фаза 1 (съём выходов ВСЕХ НКА на n, состояния А_L1OC/Б_L1OC/В неизменны) → Д_L1OC → (I,Q);
//   Фаза 2 (обновление состояний к n+1, после съёма всех источников).
// Инвариант сетки: ни одно состояние не обновляется, пока не считаны выходы всех генераторов
// того же индекса n. Порядок накопления суммы — по ВОЗРАСТАНИЮ j (Д_L1OC.8).
class SignalSourceL1OC {
public:

   explicit SignalSourceL1OC(const SourceConfigL1OC& config);

   OutputSample step();
   SampleIndex  sampleIndex() const; // n = n₀ + r — индекс СЛЕДУЮЩЕГО выдаваемого отсчёта

   // диагностика
   double       normalizationFactor() const;
   std::size_t  satelliteCount() const;

private:

   // Состояние одного НКА j (D10: состояния, индексируемые [j], — члены объекта «на источник»).
   struct SatelliteState {
      RangingCodeL1OC rangingCode;                    // таблицы ДК и кодовая фаза P_{c,j} (блок А_L1OC)
      NavMessageL1OC  navMessage;                     // сообщение b_j (блок Б_L1OC)
      CarrierNco      carrierNco;                     // фазор e_j (блок В)
      double          amplitude = 1.0;                // A_j (применяется в Г_L1OC; в η — Д_L1OC)
   };

   std::vector<SatelliteState> satellites_;           // J, упорядочено по возрастанию j
   std::vector<std::complex<double> > sourceSamples_; // буфер вкладов u_j[n] (переиспользуется)
   double normalizationFactor_ = 1.0;                 // η (Д_L1OC.1), предвычислено на запуск
   SampleIndex sampleIndex_    = 0;                   // n = n₀ + r
};
} // namespace glonass

#endif // SIGNAL_SOURCE_L1OC_H
