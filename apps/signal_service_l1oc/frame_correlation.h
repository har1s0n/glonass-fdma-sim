#ifndef SERVICE_FRAME_CORRELATION_H
#define SERVICE_FRAME_CORRELATION_H

#include <string>
#include <vector>

#include "glonass/types.h"
#include "stream_session.h"

// Корреляционные кадры дальномерных кодов (kind = acf, ccf)
//
//   ПАКФ:  R_a(τ)  = Σ_i (1−2a[i])(1−2a[(i+τ) mod N])
//   ПВКФ:  R_ab(τ) = Σ_i (1−2a[i])(1−2b[(i+τ) mod N])
//   огибающая ансамбля: env(τ) = max |R_ab(τ)| по парам a < b состава J
//   уровень: 20·lg|R(τ)/N| дБ
namespace glonass_service {
inline constexpr int corrSpanChips        = glonass::codeLengthP / 2; // полуразмах оси
inline constexpr int corrPointLimit       = 256;                      // предел точек ряда
inline constexpr double corrDbFloor       = -70.0;                    // пол уровня: lg 0 не определён
inline constexpr double corrAxisStepDb    = 10.0;
inline constexpr double corrAxisStepChips = 500.0;
inline constexpr double corrAcfAxisHighDb = 2.0;                      // запас над главным лепестком

// Бин ряда после прореживания: центр по сдвигу и границы уровней в бине
struct CorrBin {
   double tauChips  = 0.0;
   double lowDb     = 0.0;
   double highDb    = 0.0;
   double averageDb = 0.0;
};

// Ряд ПАКФ одной компоненты
struct AcfSeries {
   std::vector<CorrBin> bins;
   int                  lengthChips    = 0; // N
   int                  mainLobe       = 0; // R(0) = N
   int                  peakSidelobe   = 0; // max|R(τ)|, τ ≠ 0
   double               peakSidelobeDb = 0.0;
   std::vector<int>     sidelobeValues;     // значения R(τ), τ ≠ 0, по возрастанию
   std::vector<int>     sidelobeCounts;     // их кратности на периоде
};

// Ряд огибающей ПВКФ одной компоненты
struct CcfSeries {
   std::vector<CorrBin> bins;
   int                  lengthChips  = 0; // N
   int                  peak         = 0; // максимум огибающей
   double               peakDb       = 0.0;
   int                  shiftsAtPeak = 0; // сдвигов, на которых достигнут максимум
   int                  levelCount   = 0; // различных значений огибающей
   int                  lowest       = 0; // минимум огибающей
   double               lowestDb     = 0.0;
};

struct AcfFrame {
   int       satellite = 0;      // j — первый номер состава J
   AcfSeries codeD;
   AcfSeries codeP;
   int       decimationStep = 0; // шаг бинирования, чипов
   double    axisLowDb      = 0.0;
   double    axisHighDb     = 0.0;
};

struct CcfFrame {
   int       pairCount = 0; // |J|·(|J|−1)/2
   CcfSeries codeD;
   CcfSeries codeP;
   int       decimationStep = 0;
   double    axisLowDb      = 0.0;
   double    axisHighDb     = 0.0;
};

// ПАКФ обеих компонент первого НКА состава J
AcfFrame    computeAcfFrame(const StreamRequest& request);

// Огибающая ПВКФ по составу J; при |J| = 1 пар нет — отказ ParamError (422)
CcfFrame    computeCcfFrame(const StreamRequest& request);

std::string acfFrameJson(const AcfFrame&      frame,
                         const StreamRequest& request);
std::string acfFrameSvg(const AcfFrame&      frame,
                        const StreamRequest& request);
std::string ccfFrameJson(const CcfFrame&      frame,
                         const StreamRequest& request);
std::string ccfFrameSvg(const CcfFrame&      frame,
                        const StreamRequest& request);
} // namespace glonass_service

#endif // SERVICE_FRAME_CORRELATION_H
