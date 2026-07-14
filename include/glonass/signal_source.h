#ifndef SIGNAL_SOURCE_H
#define SIGNAL_SOURCE_H

#include <complex>
#include <cstddef>
#include <vector>

#include "glonass/carrier_nco.h"
#include "glonass/code_phase.h"
#include "glonass/nav_message.h"
#include "glonass/ranging_code.h"
#include "glonass/source_config.h"
#include "glonass/types.h"

namespace glonass {
// Источник суммарного сигнала FDMA (D2/D3). Сборка А+Б+В+Г+Д: единый блок А (общий codeTable,
// D3) + на литеру k∈K {Б_k (NavMessage), В_k (CarrierNco), Г_k (modulate)} + блок Д (Σ по K, ×η,
// → float32). Двухфазный поотсчётный шаг (§ 2.3):
//   Фаза 1 (съём выходов ВСЕХ литер на n, состояния А/Б/В неизменны) → Д → (I,Q) — отсчёт n;
//   Фаза 2 (обновление состояний А/Б/В к n+1, после съёма всех литер).
// Инвариант сетки: ни одно состояние не обновляется, пока не считаны выходы всех генераторов
// того же индекса n. Порядок суммирования в Д — по ВОЗРАСТАНИЮ k (воспроизводимость float).
class SignalSource {
public:

   // Инициализация сетки от n₀: сортировка K по возрастанию (Д.8); η = 1/√(Σ A_k²) по
   // набору A_k (Д.4); init каждой литеры (А.3/Б.4/В.4). Предусл.: |K|≥1; Σ A_k²>0; A_k≥0;
   // Fs>0; n₀≥0; на каждую литеру — В.9 (|Δf_k|+B_model ≤ Fs/2, обеспечивает CarrierNco::init).
   explicit SignalSource(const SourceConfig& config);

   OutputSample step();                      // § 2.3: съём(→Д) для n → обновление к n+1; возврат u[n] (Д.11)
   SampleIndex  sampleIndex() const;         // n = n₀ + r — индекс СЛЕДУЮЩЕГО выдаваемого отсчёта

   // диагностика (тесты § 7):
   double       normalizationFactor() const; // η (Д.1)
   std::size_t  letterCount() const;         // |K|

private:

   // Состояние одной литеры k (D10: состояния, индексируемые [k], — члены объекта «на литеру»).
   struct LetterState {
      CodePhase  codePhase;                           // P_{c,k} — кодовая фаза (блок А, на литеру)
      NavMessage navMessage;                          // сообщение b_k (блок Б, на литеру)
      CarrierNco carrierNco;                          // фазор e_k (блок В, на литеру)
      double     amplitude;                           // A_k (применяется в Г; в η — Д)
   };

   RangingCode rangingCode_;                          // единый код FDMA (D3, один экземпляр)
   std::vector<LetterState> letters_;                 // K, упорядочено по возрастанию k
   std::vector<std::complex<double> > letterSamples_; // буфер вкладов u_k[n] (переиспользуется)
   double normalizationFactor_ = 1.0;                 // η (Д.1), предвычислено на запуск
   SampleIndex sampleIndex_    = 0;                   // n = n₀ + r
};
} // namespace glonass

#endif // SIGNAL_SOURCE_H
