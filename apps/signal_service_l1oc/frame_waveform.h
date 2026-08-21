#ifndef SERVICE_FRAME_WAVEFORM_H
#define SERVICE_FRAME_WAVEFORM_H

#include <cstdint>
#include <string>
#include <vector>

#include "stream_session.h"

// Кадр осциллограммы квадратур (kind = waveform)
//
// Окно кадра задано в чипах уплотнения: на нём различимо почиповое временное уплотнение
// компонент L1OCd и L1OCp (А_L1OC.6, А_L1OC.7) и меандр пилотной компоненты (А_L1OC.10).
namespace glonass_service {
inline constexpr int waveformChips         = 16;   // окно кадра, чипов уплотнения
inline constexpr double waveformAxisFactor = 1.35; // запас оси уровней над границей η·ΣA_j
inline constexpr double waveformLabelLevel = 0.88; // высота подписей компонент, доля оси

// Чип уплотнения в окне кадра: полуинтервал по оси чипов и признак компоненты
struct WaveformZone {
   double from   = 0.0;
   double to     = 0.0;
   int    select = 0; // σ: 0 — L1OCd, 1 — L1OCp (А_L1OC.7)
};

struct WaveformFrame {
   std::vector<double>       chip;              // x[r] = r·f_T1/Fs — чип уплотнения от начала прогона
   std::vector<double>       inphase;           // I[r] = Re u[n]
   std::vector<double>       quadrature;        // Q[r] = Im u[n]
   std::vector<WaveformZone> zones;             // чипы уплотнения, попавшие в окно
   std::int64_t              sampleCount = 0;   // round(chips·Fs/f_T1)
   double                    chipSamples = 0.0; // Fs/f_T1 — отсчётов на чип
   double                    peakBound   = 0.0; // η·ΣA_j
   double                    axisLimit   = 0.0; // граница оси уровней
   double                    axisStep    = 0.0;
};

// Прогон источника на окно кадра; разметка компонент по кодовой фазе блока А_L1OC
WaveformFrame computeWaveformFrame(const StreamRequest& request);

std::string   waveformFrameJson(const WaveformFrame& frame,
                                const StreamRequest& request);
std::string   waveformFrameSvg(const WaveformFrame& frame,
                               const StreamRequest& request);
} // namespace glonass_service

#endif // SERVICE_FRAME_WAVEFORM_H
