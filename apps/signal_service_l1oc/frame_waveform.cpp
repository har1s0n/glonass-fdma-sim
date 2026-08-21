#include "frame_waveform.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>

#include "glonass/ranging_code_l1oc.h"
#include "glonass/signal_combine.h"
#include "glonass/signal_source_l1oc.h"
#include "glonass/types.h"
#include "json_writer.h"
#include "svg_canvas.h"

namespace glonass_service {
namespace {
constexpr std::size_t compositionListLimit = 4; // до скольких НКА состав выводится перечислением

// Значение, общее для всего состава; при разнобое подпись параметра не выводится
bool commonValue(const std::vector<double>& values, double& value) {
   if (values.empty()) {
      return false;
   }
   value = values.front();

   for (const double item : values) {
      if (item != value) {
         return false;
      }
   }
   return true;
}

// Значение с указанным числом знаков; у целых дробная часть не печатается
std::string numberUpTo(double value, int digits) {
   return numberRu(value, (value == std::floor(value)) ? 0 : digits);
}

// Зоны компонент в блоке render: полуинтервал по оси чипов и признак компоненты
std::string zoneArray(const std::vector<WaveformZone>& zones) {
   std::string out = "[";

   for (std::size_t i = 0; i < zones.size(); ++i) {
      JsonObject zone;

      zone.addDouble("from", zones[i].from);
      zone.addDouble("to",   zones[i].to);
      zone.addInt("select", zones[i].select);

      if (i > 0) {
         out += ", ";
      }
      out += zone.str();
   }
   return out + "]";
}

// номер НКА задаёт дальномерный код и вид осциллограммы, иначе мощностью множества
std::string compositionOf(const std::vector<int>& satellites) {
   if (satellites.size() > compositionListLimit) {
      return "|J| = " + std::to_string(satellites.size());
   }
   std::string out = "J = {";

   for (std::size_t i = 0; i < satellites.size(); ++i) {
      if (i > 0) {
         out += ", ";
      }
      out += std::to_string(satellites[i]);
   }
   return out + "}";
}

// Строка параметров прогона, общая часть шапки кадра
std::string scenarioLine(const StreamRequest& request) {
   const double eta            = glonass::normalizationFactor(request.amplitudes);    // Д_L1OC.1
   const std::int64_t residual = glonass_params::residualFreq(request.referenceFreq); // Δf_j (В.1)
   double amplitude            = 0.0;
   double phase                = 0.0;
   std::string line            = compositionOf(request.satellites);

   if (commonValue(request.amplitudes, amplitude)) {
      line += " · A_j = " + numberUpTo(amplitude, 3);
   }
   line += " · Fs = " + numberRu(static_cast<double> (request.sampleRate) / 1.0e6, 1) + " МГц";
   line += " · f₀ = " + numberRu(static_cast<double> (request.referenceFreq) / 1.0e6, 3) + " МГц";
   line += " · n₀ = " + numberRu(static_cast<double> (request.startSample));
   line += " · η = " + numberUpTo(eta, 6);
   line += " · Δf = " + numberRu(static_cast<double> (residual)) + " Гц";

   if (commonValue(request.initialPhases, phase)) {
      line += " · φ₀ = " + numberUpTo(phase, 3) + " рад";
   }
   return line;
}
} // namespace

WaveformFrame computeWaveformFrame(const StreamRequest& request) {
   WaveformFrame frame;

   frame.chipSamples = static_cast<double> (request.sampleRate)
                       / static_cast<double> (glonass::chipRateL1OC);
   frame.sampleCount = std::llround(static_cast<double> (waveformChips) * frame.chipSamples);
   frame.peakBound   = peakBound(request.amplitudes);
   frame.axisLimit   = waveformAxisFactor * frame.peakBound;
   frame.axisStep    = niceStep(2.0 * frame.axisLimit, 6);

   // Прогон источника: только окно кадра, накопления сверх окна нет
   glonass::SignalSourceL1OC source(sourceConfigOf(request));
   const std::size_t count = static_cast<std::size_t> (frame.sampleCount);

   frame.chip.reserve(count);
   frame.inphase.reserve(count);
   frame.quadrature.reserve(count);

   for (std::int64_t index = 0; index < frame.sampleCount; ++index) {
      const glonass::OutputSample sample = source.step();

      frame.chip.push_back(static_cast<double> (index)
                           * static_cast<double> (glonass::chipRateL1OC)
                           / static_cast<double> (request.sampleRate));
      frame.inphase.push_back(static_cast<double> (sample.real()));
      frame.quadrature.push_back(static_cast<double> (sample.imag()));
   }

   // Разметка компонент — из блока А_L1OC: аккумулятор кодовой фазы, начатый с n₀ (А_L1OC.5).
   // Таблицы ДК не нужны: σ определяется одной кодовой фазой (А_L1OC.6, А_L1OC.7). Кодовая фаза
   // общая для состава: φ_{c0,j} = 0 (§ 0.1 поз.47).
   glonass::RangingCodeL1OC marker;

   marker.initCodePhaseAtSampleL1OC(static_cast<glonass::SampleIndex> (request.startSample),
                                    request.sampleRate);
   const int firstChip     = marker.multiplexChipIndex();       // m[n₀]
   const double chipOffset = static_cast<double> (marker.codePhaseAccumulator())
                             / static_cast<double> (request.sampleRate)
                             - static_cast<double> (firstChip); // дробная часть чипа в начале окна
   const double window = static_cast<double> (waveformChips);
   int chipIndex       = firstChip;
   double from         = 0.0;

   for (double edge = 1.0 - chipOffset; from < window; edge += 1.0) {
      WaveformZone zone;

      zone.from   = from;
      zone.to     = std::min(edge, window);
      zone.select = chipIndex % 2; // чётность m сохраняется при переносе: M = 8184 чётно
      frame.zones.push_back(zone);
      from = zone.to;
      ++chipIndex;
   }
   return frame;
}

std::string waveformFrameJson(const WaveformFrame& frame, const StreamRequest& request) {
   JsonObject signal;

   signal.addInt("sampleRateHz",    request.sampleRate);
   signal.addInt("referenceFreqHz", request.referenceFreq);
   signal.addInt("startSample",     request.startSample);
   signal.addRaw("satellites",    jsonIntegerArray(request.satellites));
   signal.addRaw("amplitudes",    jsonNumberArray(request.amplitudes, "%g"));
   signal.addRaw("initialPhases", jsonNumberArray(request.initialPhases, "%g"));
   signal.addInt("residualFreqHz", glonass_params::residualFreq(request.referenceFreq));
   JsonObject window;

   window.addInt("chips",       waveformChips);
   window.addInt("sampleCount", frame.sampleCount);
   window.addInt("chipRateHz",  glonass::chipRateL1OC);
   window.addDouble("chipSamples",    frame.chipSamples);
   window.addDouble("chipDurationNs", 1.0e9 / static_cast<double> (glonass::chipRateL1OC));
   window.addDouble("durationUs",     static_cast<double> (waveformChips) * 1.0e6
                    / static_cast<double> (glonass::chipRateL1OC));
   JsonObject render;

   render.addString("xLabel", "Чип уплотнения от начала прогона");
   render.addString("xUnit",  "чип");
   render.addDouble("xLow",  0.0);
   render.addDouble("xHigh", static_cast<double> (waveformChips));
   render.addDouble("xStep", 2.0);
   render.addString("yLabel", "Нормированный отсчёт u[n]");
   render.addDouble("yLow",      -frame.axisLimit);
   render.addDouble("yHigh",     frame.axisLimit);
   render.addDouble("yStep",     frame.axisStep);
   render.addDouble("peakBound", frame.peakBound);
   render.addString("zoneRule", "заливка чипов компоненты L1OCp (σ = 1)");
   render.addRaw("zones", zoneArray(frame.zones));
   JsonObject series;

   series.addRaw("chip",       jsonNumberArray(frame.chip, "%.6f"));
   series.addRaw("inphase",    jsonNumberArray(frame.inphase, "%.9g"));
   series.addRaw("quadrature", jsonNumberArray(frame.quadrature, "%.9g"));
   JsonObject root;

   root.addString("kind",  "waveform");
   root.addString("title", "Осциллограмма квадратур I и Q в чиповом масштабе");
   root.addRaw("signal", signal.str());
   root.addRaw("window", window.str());
   root.addRaw("render", render.str());
   root.addRaw("series", series.str());
   return root.str();
}

std::string waveformFrameSvg(const WaveformFrame& frame, const StreamRequest& request) {
   const std::string window = "Окно " + std::to_string(waveformChips) + " чипов уплотнения = "
                              + numberRu(static_cast<double> (waveformChips) * 1.0e6
                                         / static_cast<double> (glonass::chipRateL1OC), 2)
                              + " мкс = " + numberRu(static_cast<double> (frame.sampleCount))
                              + " отсчётов · чип "
                              + numberRu(1.0e9 / static_cast<double> (glonass::chipRateL1OC), 1)
                              + " нс";
   SvgCanvas canvas("Осциллограмма квадратур I и Q в чиповом масштабе",
                    { scenarioLine(request), window }, "Чип уплотнения от начала прогона",
                    "Нормированный отсчёт u[n]",
                    "Осциллограмма квадратур суммарного сигнала L1OC: почиповое временное "
                    "уплотнение компонент L1OCd и L1OCp. signal-service-l1oc, "
                    "ИКД ГЛОНАСС L1OC ред. 1.0 (2016).");

   canvas.setX(0.0, static_cast<double> (waveformChips), 2.0, 2);
   canvas.setY(-frame.axisLimit, frame.axisLimit, frame.axisStep, 5);

   for (const WaveformZone& zone : frame.zones) { // зоны пилотной компоненты
      if (zone.select == 1) {
         canvas.band(zone.from, zone.to, svg::series3, 0.10);
      }
   }
   std::vector<std::pair<double, double> > inphase;
   std::vector<std::pair<double, double> > quadrature;

   inphase.reserve(frame.chip.size());
   quadrature.reserve(frame.chip.size());

   for (std::size_t i = 0; i < frame.chip.size(); ++i) {
      inphase.emplace_back(frame.chip[i], frame.inphase[i]);
      quadrature.emplace_back(frame.chip[i], frame.quadrature[i]);
   }
   canvas.steps(inphase, svg::series1, 1.8);
   canvas.polyline(quadrature, svg::series2, 1.6, svg::dashLine);

   // Подписи компонент — над первым чипом каждой из них
   for (int select = 0; select <= 1; ++select) {
      const auto zone = std::find_if(frame.zones.begin(), frame.zones.end(),
                                     [select](const WaveformZone& item) {
            return item.select == select;
         });

      if (zone != frame.zones.end()) {
         canvas.labelAt((zone->from + zone->to) / 2.0, waveformLabelLevel * frame.axisLimit,
                        (select == 0) ? "L1OCd" : "L1OCp", 11,
                        (select == 0) ? svg::textSecond : svg::series3);
      }
   }
   canvas.legend({ LegendRow{ svg::series1, svg::dashSolid, "Квадратура I" },
                   LegendRow{ svg::series2, svg::dashLine, "Квадратура Q" },
                   LegendRow{ svg::series3, svg::dashFill, "Чипы компоненты L1OCp" } });
   return canvas.str();
}
} // namespace glonass_service
