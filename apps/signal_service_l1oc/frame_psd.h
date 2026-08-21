#ifndef SERVICE_FRAME_PSD_H
#define SERVICE_FRAME_PSD_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "stream_session.h"

// Кадр спектральной плотности мощности (kind = psd)
//
// Правила отображения выводятся в ответе JSON блоком render, по нему потребитель строит тот же кадр из числовых рядов.
namespace glonass_service {
inline constexpr std::size_t psdSegmentLength = 8192;              // длина сегмента Уэлча
inline constexpr int psdSegmentCount          = 32;                // число сегментов
inline constexpr std::int64_t psdSampleCount  =
   static_cast<std::int64_t> (psdSegmentLength) * psdSegmentCount; // длительность прогона
inline constexpr int framePointLimit        = 2000;                // предел точек на ряд
inline constexpr int psdSmoothingHalfWindow = 16;                  // окно нормировки, бинов
inline constexpr double frameAxisStepDb     = 10.0;                // шаг оси уровней
inline constexpr double frameAxisMarginDb   = 5.0;                 // запас границ оси

// Бин ряда после прореживания: центр по частоте и границы уровней в бине
struct PsdBin {
   double centerHz  = 0.0;
   double lowDb     = 0.0;
   double highDb    = 0.0;
   double averageDb = 0.0;
};

struct PsdFrame {
   std::vector<PsdBin> bins;
   double              resolutionHz   = 0.0; // Fs / длина сегмента
   double              binStepHz      = 0.0; // шаг центров после прореживания
   int                 decimationStep = 1;   // ceil(длина спектра / предел точек)
   double              axisLowDb      = 0.0;
   double              axisHighDb     = 0.0;
};

// Прогон источника на psdSampleCount отсчётов и оценка спектральной плотности методом Уэлча
PsdFrame    computePsdFrame(const StreamRequest& request);

std::string psdFrameJson(const PsdFrame&      frame,
                         const StreamRequest& request);
std::string psdFrameSvg(const PsdFrame&      frame,
                        const StreamRequest& request);
} // namespace glonass_service

#endif // SERVICE_FRAME_PSD_H
