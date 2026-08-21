#ifndef SERVICE_FRAME_LEVEL_H
#define SERVICE_FRAME_LEVEL_H

#include <cstdint>
#include <string>
#include <vector>

#include "stream_session.h"

// Кадр гистограммы мгновенных значений (kind = level)
//
// Наблюдаемый блок: Д_L1OC: нормировка η, аналитическая граница шкалы η·ΣA_j, пик-фактор.
namespace glonass_service {
inline constexpr int levelBinCount             = 128;    // корзин гистограммы
inline constexpr std::int64_t levelSampleCount = 262144; // та же длительность, что кадр psd
inline constexpr double levelAxisStepPercent   = 10.0;   // шаг оси долей

struct LevelFrame {
   std::vector<double>       edges;                      // границы корзин, levelBinCount + 1 значение
   std::vector<std::int64_t> counts;                     // отсчётов в корзине
   std::vector<double>       shares;                     // доля отсчётов в корзине, %
   double                    eta             = 0.0;      // η (Д_L1OC.1)
   double                    limit           = 0.0;      // граница шкалы (Д_L1OC.3)
   double                    rms             = 0.0;      // среднеквадратичное значение квадратуры I
   double                    peak            = 0.0;      // max |I| по прогону
   double                    crestFactor     = 0.0;      // пик-фактор
   double                    crestFactorDb   = 0.0;
   double                    axisHighPercent = 0.0;      // верх оси долей
   double                    axisStepValue   = 0.0;      // шаг оси значений
};

// Прогон источника на levelSampleCount отсчётов; учитывается квадратура I
LevelFrame  computeLevelFrame(const StreamRequest& request);

std::string levelFrameJson(const LevelFrame&    frame,
                           const StreamRequest& request);
std::string levelFrameSvg(const LevelFrame&    frame,
                          const StreamRequest& request);
} // namespace glonass_service

#endif // SERVICE_FRAME_LEVEL_H
