#include "frame_level.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

#include "glonass/signal_combine.h"
#include "glonass/signal_source_l1oc.h"
#include "glonass/types.h"
#include "json_writer.h"
#include "svg_canvas.h"

namespace glonass_service {
namespace {
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
} // namespace

LevelFrame computeLevelFrame(const StreamRequest& request) {
   LevelFrame frame;

   frame.eta   = glonass::normalizationFactor(request.amplitudes); // Д_L1OC.1
   frame.limit = peakBound(request.amplitudes);                    // η·ΣA_j
   frame.edges.reserve(static_cast<std::size_t> (levelBinCount) + 1U);

   for (int bin = 0; bin <= levelBinCount; ++bin) {
      frame.edges.push_back(-frame.limit + 2.0 * frame.limit * static_cast<double> (bin)
                            / static_cast<double> (levelBinCount));
   }
   frame.counts.assign(static_cast<std::size_t> (levelBinCount), 0);

   // Прогон источника: отсчёты идут подряд, запись целиком не накапливается
   glonass::SignalSourceL1OC source(sourceConfigOf(request));
   double sumSquares = 0.0;

   for (std::int64_t index = 0; index < levelSampleCount; ++index) {
      const glonass::OutputSample sample = source.step();
      const double value                 = static_cast<double> (sample.real()); // квадратура I

      sumSquares += value * value;
      frame.peak  = std::max(frame.peak, std::fabs(value));

      // Корзина по равномерной шкале; значение на границе относится в верхнюю корзину, значение
      // на границе шкалы (|u| = η·ΣA_j с точностью float32) — в крайнюю
      const int bin = static_cast<int> ((value + frame.limit) / (2.0 * frame.limit)
                                        * static_cast<double> (levelBinCount));

      frame.counts[static_cast<std::size_t> (std::min(levelBinCount - 1, std::max(0, bin)))] += 1;
   }
   frame.rms = std::sqrt(sumSquares / static_cast<double> (levelSampleCount));
   frame.shares.reserve(static_cast<std::size_t> (levelBinCount));
   double highest = 0.0;

   for (const std::int64_t count : frame.counts) {
      const double share = static_cast<double> (count) / static_cast<double> (levelSampleCount)
                           * 100.0;

      frame.shares.push_back(share);
      highest = std::max(highest, share);
   }
   frame.crestFactor   = (frame.rms > 0.0) ? (frame.peak / frame.rms) : 0.0;
   frame.crestFactorDb = (frame.crestFactor > 0.0) ? (20.0 * std::log10(frame.crestFactor)) : 0.0;
   frame.axisHighPercent = std::ceil(highest / levelAxisStepPercent) * levelAxisStepPercent;
   frame.axisStepValue   = niceStep(2.0 * frame.limit, 8);
   return frame;
}

std::string levelFrameJson(const LevelFrame& frame, const StreamRequest& request) {
   JsonObject signal;

   signal.addInt("sampleRateHz",    request.sampleRate);
   signal.addInt("referenceFreqHz", request.referenceFreq);
   signal.addInt("startSample",     request.startSample);
   signal.addRaw("satellites",    jsonIntegerArray(request.satellites));
   signal.addRaw("amplitudes",    jsonNumberArray(request.amplitudes, "%g"));
   signal.addRaw("initialPhases", jsonNumberArray(request.initialPhases, "%g"));
   signal.addInt("residualFreqHz", glonass_params::residualFreq(request.referenceFreq));
   JsonObject metrics;

   metrics.addString("quadrature", "I");
   metrics.addInt("sampleCount", levelSampleCount);
   metrics.addInt("binCount",    levelBinCount);
   metrics.addDouble("eta",   frame.eta);
   metrics.addDouble("limit", frame.limit);
   metrics.addString("limitRule", "η·ΣA_j");
   metrics.addDouble("rms",           frame.rms);
   metrics.addDouble("peak",          frame.peak);
   metrics.addDouble("crestFactor",   frame.crestFactor);
   metrics.addDouble("crestFactorDb", frame.crestFactorDb);
   JsonObject render;

   render.addString("xLabel", "Значение отсчёта I");
   render.addDouble("xLow",  -frame.limit);
   render.addDouble("xHigh", frame.limit);
   render.addDouble("xStep", frame.axisStepValue);
   render.addString("yLabel", "Доля отсчётов, %");
   render.addString("yUnit",  "%");
   render.addDouble("yLow",  0.0);
   render.addDouble("yHigh", frame.axisHighPercent);
   render.addDouble("yStep", levelAxisStepPercent);
   render.addString("rule", "binUniform");
   JsonObject series;

   series.addRaw("edge",         jsonNumberArray(frame.edges, "%.9g"));
   series.addRaw("count",        jsonIntegerArray(frame.counts));
   series.addRaw("sharePercent", jsonNumberArray(frame.shares, "%.6f"));
   JsonObject root;

   root.addString("kind",  "level");
   root.addString("title", "Гистограмма мгновенных значений суммарного сигнала");
   root.addRaw("signal",  signal.str());
   root.addRaw("metrics", metrics.str());
   root.addRaw("render",  render.str());
   root.addRaw("series",  series.str());
   return root.str();
}

std::string levelFrameSvg(const LevelFrame& frame, const StreamRequest& request) {
   double amplitude    = 0.0;
   double phase        = 0.0;
   std::string scenario = "|J| = " + std::to_string(request.satellites.size());

   if (commonValue(request.amplitudes, amplitude)) {
      scenario += " · A_j = " + numberUpTo(amplitude, 3);
   }
   scenario += " · Fs = " + numberRu(static_cast<double> (request.sampleRate) / 1.0e6, 1) + " МГц";
   scenario += " · f₀ = " + numberRu(static_cast<double> (request.referenceFreq) / 1.0e6, 3)
               + " МГц";
   scenario += " · n₀ = " + numberRu(static_cast<double> (request.startSample));
   scenario += " · отсчётов " + numberRu(static_cast<double> (levelSampleCount));
   scenario += " · корзин " + std::to_string(levelBinCount);
   std::string normalization = "Квадратура I · Δf = "
                               + numberRu(static_cast<double> (glonass_params::residualFreq(
                                                                  request.referenceFreq)))
                               + " Гц";

   if (commonValue(request.initialPhases, phase)) {
      normalization += " · φ₀ = " + numberUpTo(phase, 3) + " рад";
   }
   normalization += " · нормировка Д_L1OC.1: η = 1/√(ΣA_j²), границы шкалы ±η·ΣA_j";
   SvgCanvas canvas("Гистограмма мгновенных значений суммарного сигнала",
                    { scenario, normalization }, "Значение отсчёта I", "Доля отсчётов, %",
                    "Распределение мгновенных значений суммарного сигнала L1OC после нормировки "
                    "блока Д_L1OC. signal-service-l1oc, ИКД ГЛОНАСС L1OC ред. 1.0 (2016).");

   canvas.setX(-frame.limit, frame.limit, frame.axisStepValue, 4);
   canvas.setY(0.0, frame.axisHighPercent, levelAxisStepPercent, 5);
   canvas.bars(frame.edges, frame.shares, svg::series1, 0.8);
   canvas.verticalLine(-frame.rms, svg::series2, svg::dashLine);
   canvas.verticalLine(frame.rms,  svg::series2, svg::dashLine);
   canvas.verticalLine(frame.limit, svg::markColor, svg::dashMark, "η·ΣA_j", false, true);
   canvas.verticalLine(-frame.limit, svg::markColor, svg::dashMark, "−η·ΣA_j");
   canvas.note(svg::plotLeft + 16, svg::plotTop + 26, "η = " + numberUpTo(frame.eta, 6), 12);
   canvas.note(svg::plotLeft + 16, svg::plotTop + 44,
               "среднеквадратичное значение = " + numberRu(frame.rms, 4), 12);
   canvas.note(svg::plotLeft + 16, svg::plotTop + 62,
               "пик = " + numberRu(frame.peak, 4) + " · пик-фактор = "
               + numberRu(frame.crestFactor, 3) + " = " + numberRu(frame.crestFactorDb, 2)
               + " дБ", 12);
   canvas.legend({ LegendRow{ svg::series1, svg::dashSolid, "Доля отсчётов в корзине" },
                   LegendRow{ svg::series2, svg::dashLine, "±среднеквадратичное значение" },
                   LegendRow{ svg::markColor, svg::dashMark, "Граница шкалы η·ΣA_j" } });
   return canvas.str();
}
} // namespace glonass_service
