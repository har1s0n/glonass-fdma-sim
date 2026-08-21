#include "frame_psd.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <numbers>

#include "fft_radix2.h"
#include "glonass/signal_source_l1oc.h"
#include "glonass/types.h"
#include "json_writer.h"
#include "svg_canvas.h"

namespace glonass_service {
namespace {
constexpr double dbFloor = -200.0; // уровень для нулевой оценки: логарифм не определён

// Окно Хэнна и его энергия (множитель нормировки оценки Уэлча)
std::vector<double> hannWindow(std::size_t length) {
   std::vector<double> window(length);

   for (std::size_t i = 0; i < length; ++i) {
      window[i] = 0.5 - 0.5 * std::cos(2.0 * std::numbers::pi * static_cast<double> (i)
                                       / static_cast<double> (length));
   }
   return window;
}

// Скользящее среднее полушириной half; края усредняются по доступным отсчётам
std::vector<double> movingAverage(const std::vector<double>& values, int half) {
   std::vector<double> out(values.size());

   for (std::size_t i = 0; i < values.size(); ++i) {
      const std::size_t from = (static_cast<int> (i) > half) ? (i - static_cast<std::size_t> (half)) : 0;
      const std::size_t to   = std::min(values.size(), i + static_cast<std::size_t> (half) + 1);
      double sum             = 0.0;

      for (std::size_t k = from; k < to; ++k) {
         sum += values[k];
      }
      out[i] = sum / static_cast<double> (to - from);
   }
   return out;
}

double toDb(double value, double reference) {
   return (value > 0.0) ? (10.0 * std::log10(value / reference)) : dbFloor;
}

std::string joinNumbers(const std::vector<double>& values, const char* pattern) {
   std::string out = "[";
   char buffer[48];

   for (std::size_t i = 0; i < values.size(); ++i) {
      if (i > 0) {
         out += ", ";
      }
      std::snprintf(buffer, sizeof(buffer), pattern, values[i]);
      out += buffer;
   }
   return out + "]";
}

std::string joinIntegers(const std::vector<int>& values) {
   std::string out = "[";

   for (std::size_t i = 0; i < values.size(); ++i) {
      if (i > 0) {
         out += ", ";
      }
      out += std::to_string(values[i]);
   }
   return out + "]";
}
} // namespace

PsdFrame computePsdFrame(const StreamRequest& request) {
   const std::size_t segment        = psdSegmentLength;
   const std::vector<double> window = hannWindow(segment);
   double windowEnergy              = 0.0;

   for (const double weight : window) {
      windowEnergy += weight * weight;
   }
   windowEnergy /= static_cast<double> (segment);

   // Прогон источника: сегменты идут подряд, перекрытия нет; запись целиком не накапливается
   glonass::SignalSourceL1OC source(sourceConfigOf(request));
   std::vector<double> spectrum(segment, 0.0);
   std::vector<std::complex<double> > buffer(segment);

   for (int index = 0; index < psdSegmentCount; ++index) {
      for (std::size_t i = 0; i < segment; ++i) {
         const glonass::OutputSample sample = source.step();

         buffer[i] = std::complex<double> (static_cast<double> (sample.real()) * window[i],
                                           static_cast<double> (sample.imag()) * window[i]);
      }
      fftRadix2(buffer);

      for (std::size_t i = 0; i < segment; ++i) {
         spectrum[i] += std::norm(buffer[i]);
      }
   }
   const double scale = 1.0 / (static_cast<double> (psdSegmentCount) * static_cast<double> (segment)
                               * windowEnergy * static_cast<double> (request.sampleRate));

   for (double& value : spectrum) {
      value *= scale;
   }

   // Порядок частот от −Fs/2 к +Fs/2
   const std::size_t   half = segment / 2;
   std::vector<double> shifted(segment);

   for (std::size_t i = 0; i < segment; ++i) {
      shifted[i] = spectrum[(i + half) % segment];
   }

   const std::vector<double> smoothed = movingAverage(shifted, psdSmoothingHalfWindow);
   const double reference             = *std::max_element(smoothed.begin(), smoothed.end());
   std::vector<double> levels(segment);

   for (std::size_t i = 0; i < segment; ++i) {
      levels[i] = toDb(shifted[i], reference);
   }
   PsdFrame frame;

   frame.resolutionHz   = static_cast<double> (request.sampleRate) / static_cast<double> (segment);
   frame.decimationStep = static_cast<int> ((segment + framePointLimit - 1) / framePointLimit);

   // Прореживание бинированием: сохраняются границы значений в бине
   for (std::size_t start = 0; start < segment; start += static_cast<std::size_t> (frame.decimationStep)) {
      const std::size_t stop = std::min(segment, start + static_cast<std::size_t> (frame.decimationStep));
      PsdBin bin;
      double sum = 0.0;

      bin.lowDb  = levels[start];
      bin.highDb = levels[start];

      for (std::size_t i = start; i < stop; ++i) {
         bin.lowDb  = std::min(bin.lowDb, levels[i]);
         bin.highDb = std::max(bin.highDb, levels[i]);
         sum       += levels[i];
      }
      bin.averageDb = sum / static_cast<double> (stop - start);
      bin.centerHz  = (static_cast<double> (start + (stop - start) / 2) - static_cast<double> (half))
                      * frame.resolutionHz;
      frame.bins.push_back(bin);
   }

   if (frame.bins.size() > 1) {
      frame.binStepHz = frame.bins[1].centerHz - frame.bins[0].centerHz;
   }
   double lowest  = frame.bins.front().lowDb;
   double highest = frame.bins.front().highDb;

   for (const PsdBin& bin : frame.bins) {
      lowest  = std::min(lowest, bin.lowDb);
      highest = std::max(highest, bin.highDb);
   }
   frame.axisLowDb  = std::floor((lowest - frameAxisMarginDb) / frameAxisStepDb) * frameAxisStepDb;
   frame.axisHighDb = std::ceil((highest + frameAxisMarginDb) / frameAxisStepDb) * frameAxisStepDb;
   return frame;
}

std::string psdFrameJson(const PsdFrame& frame, const StreamRequest& request) {
   std::vector<double> centers;
   std::vector<double> low;
   std::vector<double> high;
   std::vector<double> average;

   centers.reserve(frame.bins.size());
   low.reserve(frame.bins.size());
   high.reserve(frame.bins.size());
   average.reserve(frame.bins.size());

   for (const PsdBin& bin : frame.bins) {
      centers.push_back(bin.centerHz);
      low.push_back(bin.lowDb);
      high.push_back(bin.highDb);
      average.push_back(bin.averageDb);
   }
   JsonObject signal;

   signal.addInt("sampleRateHz",    request.sampleRate);
   signal.addInt("referenceFreqHz", request.referenceFreq);
   signal.addInt("startSample",     request.startSample);
   signal.addRaw("satellites", joinIntegers(request.satellites));
   signal.addRaw("amplitudes", joinNumbers(request.amplitudes, "%g"));
   JsonObject estimate;

   estimate.addString("method", "welch");
   estimate.addInt("segmentLength", static_cast<std::int64_t> (psdSegmentLength));
   estimate.addInt("segmentCount",  psdSegmentCount);
   estimate.addString("window", "hann");
   estimate.addInt("overlap",     0);
   estimate.addInt("sampleCount", psdSampleCount);
   estimate.addDouble("resolutionHz", frame.resolutionHz);
   estimate.addString("normalization", "максимум сглаженной оценки");
   estimate.addInt("smoothingHalfWindow", psdSmoothingHalfWindow);
   JsonObject decimation;

   decimation.addString("rule", "binMinMax");
   decimation.addInt("limit",  framePointLimit);
   decimation.addInt("step",   frame.decimationStep);
   decimation.addInt("points", static_cast<std::int64_t> (frame.bins.size()));
   JsonObject render;

   render.addString("xLabel", "Частота относительно f₀");
   render.addString("xUnit",  "МГц");
   render.addDouble("xLowHz",  -static_cast<double> (request.sampleRate) / 2.0);
   render.addDouble("xHighHz", static_cast<double> (request.sampleRate) / 2.0);
   render.addDouble("xStepHz", 2.0e6);
   render.addString("yLabel", "СПМ, дБ отн. максимума");
   render.addString("yUnit",  "дБ");
   render.addDouble("yLow",  frame.axisLowDb);
   render.addDouble("yHigh", frame.axisHighDb);
   render.addDouble("yStep", frameAxisStepDb);
   render.addRaw("decimation", decimation.str());
   render.addInt("modelBandwidthHz", glonass::modelBandwidthL1OC);
   JsonObject series;

   series.addRaw("centerHz", joinNumbers(centers, "%.1f"));
   series.addRaw("minDb",    joinNumbers(low, "%.3f"));
   series.addRaw("maxDb",    joinNumbers(high, "%.3f"));
   series.addRaw("avgDb",    joinNumbers(average, "%.3f"));
   JsonObject root;

   root.addString("kind",  "psd");
   root.addString("title", "Спектральная плотность мощности суммарного сигнала L1OC");
   root.addRaw("signal",   signal.str());
   root.addRaw("estimate", estimate.str());
   root.addRaw("render",   render.str());
   root.addRaw("series",   series.str());
   return root.str();
}

std::string psdFrameSvg(const PsdFrame& frame, const StreamRequest& request) {
   const double halfRateMHz  = static_cast<double> (request.sampleRate) / 2.0e6;
   const double bandwidthMHz = static_cast<double> (glonass::modelBandwidthL1OC) / 1.0e6;
   double sumSquares         = 0.0;

   for (const double amplitude : request.amplitudes) {
      sumSquares += amplitude * amplitude;
   }
   const double eta = (sumSquares > 0.0) ? (1.0 / std::sqrt(sumSquares)) : 1.0;
   char line[512];

   std::snprintf(line, sizeof(line),
                 "|J| = %zu · Fs = %s МГц · f₀ = %s МГц · n₀ = %s · η = %s",
                 request.satellites.size(),
                 numberRu(static_cast<double> (request.sampleRate) / 1.0e6,    1).c_str(),
                 numberRu(static_cast<double> (request.referenceFreq) / 1.0e6, 3).c_str(),
                 numberRu(static_cast<double> (request.startSample)).c_str(),
                 numberRu(eta,                                                 6).c_str());
   const std::string scenario = line;

   std::snprintf(line, sizeof(line),
                 "Оценка Уэлча: %d сегмента × %s отсч., окно Хэнна · бинирование min/max: "
                 "%s точек (предел %s)", psdSegmentCount,
                 numberRu(static_cast<double> (psdSegmentLength)).c_str(),
                 numberRu(static_cast<double> (frame.bins.size())).c_str(),
                 numberRu(framePointLimit).c_str());
   const std::string estimate = line;
   SvgCanvas canvas("Спектральная плотность мощности суммарного сигнала L1OC",
                    { scenario, estimate }, "Частота относительно f₀, МГц",
                    "СПМ, дБ отн. максимума",
                    "Оценка спектральной плотности мощности по отсчётам цифровой модели "
                    "навигационного сигнала ГЛОНАСС L1OC. signal-service-l1oc, "
                    "ИКД ГЛОНАСС L1OC ред. 1.0 (2016).");

   canvas.setX(-halfRateMHz, halfRateMHz, 2.0, 4);
   canvas.setY(frame.axisLowDb, frame.axisHighDb, frameAxisStepDb, 5);
   canvas.band(-bandwidthMHz, bandwidthMHz);
   std::vector<Bin> bins;
   std::vector<std::pair<double, double> > average;

   bins.reserve(frame.bins.size());
   average.reserve(frame.bins.size());

   for (const PsdBin& bin : frame.bins) {
      const double x = bin.centerHz / 1.0e6;

      bins.push_back(Bin{ x, bin.lowDb, bin.highDb });
      average.emplace_back(x, bin.averageDb);
   }
   canvas.envelope(bins, svg::series1, 0.18);
   canvas.polyline(average, svg::series1, 1.6);
   canvas.verticalLine(-bandwidthMHz, svg::markColor, svg::dashMark,
                       "−B_model", false, true);
   canvas.verticalLine(bandwidthMHz,  svg::markColor, svg::dashMark, "+B_model");
   canvas.note(svg::plotLeft,  svg::plotBottom + 34, "−Fs/2", 10,
               svg::textSecond, "middle");
   canvas.note(svg::plotRight, svg::plotBottom + 34, "+Fs/2", 10, svg::textSecond, "middle");
   std::snprintf(line, sizeof(line), "Полоса модели B_model = %s МГц",
                 numberRu(bandwidthMHz, 3).c_str());
   canvas.legend({ LegendRow{ svg::series1, svg::dashSolid, "Оценка по отсчётам модели (Уэлч)" },
                   LegendRow{ svg::markColor, svg::dashMark, line } });
   return canvas.str();
}
} // namespace glonass_service
