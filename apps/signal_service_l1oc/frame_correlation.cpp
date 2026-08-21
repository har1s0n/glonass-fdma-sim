#include "frame_correlation.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "glonass/ranging_code_l1oc.h"
#include "json_writer.h"
#include "request_params_l1oc.h"
#include "svg_canvas.h"

namespace glonass_service {
namespace {
constexpr std::size_t sidelobeListLimit = 4; // до скольких значений набор перечисляется в подписи

// Символы кода в 64-разрядных словах: символ i — разряд i mod 64 слова i/64. При repeats = 2
// получается удвоенная последовательность: циклический сдвиг сводится к чтению окна с разряда τ
std::vector<std::uint64_t> packCode(const glonass::Bit* symbols, int length, int repeats) {
   const std::size_t bits  = static_cast<std::size_t> (length) * static_cast<std::size_t> (repeats);
   const std::size_t words = bits / 64U + 2U; // запасное слово: чтение окна берёт следующее
   std::vector<std::uint64_t> packed(words, 0U);

   for (std::size_t i = 0; i < bits; ++i) {
      if (symbols[i % static_cast<std::size_t> (length)] != glonass::Bit{ 0 }) {
         packed[i / 64U] |= (std::uint64_t{ 1 } << (i % 64U));
      }
   }
   return packed;
}

// R(τ) = N − 2·popcount(a XOR окно удвоенного b с разряда τ) — счёт в алфавите {+1, −1}
int correlationAt(const std::vector<std::uint64_t>& a,
                  const std::vector<std::uint64_t>& doubled, int tau, int length) {
   const std::size_t words      = (static_cast<std::size_t> (length) + 63U) / 64U;
   const std::size_t shiftWords = static_cast<std::size_t> (tau) / 64U;
   const unsigned    shiftBits  = static_cast<unsigned> (tau) % 64U;
   const unsigned    tailBits   = static_cast<unsigned> (length) % 64U;
   int differing                = 0;

   for (std::size_t k = 0; k < words; ++k) {
      std::uint64_t value = doubled[shiftWords + k] >> shiftBits;

      if (shiftBits != 0U) {
         value |= doubled[shiftWords + k + 1U] << (64U - shiftBits);
      }
      std::uint64_t difference = value ^ a[k];

      if ((k + 1U == words) && (tailBits != 0U)) {
         difference &= (~std::uint64_t{ 0 } >> (64U - tailBits));
      }
      differing += std::popcount(difference);
   }
   return length - 2 * differing;
}

std::vector<int> correlationVector(const std::vector<std::uint64_t>& a,
                                   const std::vector<std::uint64_t>& doubled, int length) {
   std::vector<int> values(static_cast<std::size_t> (length), 0);

   for (int tau = 0; tau < length; ++tau) {
      values[static_cast<std::size_t> (tau)] = correlationAt(a, doubled, tau, length);
   }
   return values;
}

// Уровень по мощности относительно длины кода; нулевое значение логарифма не имеет — пол шкалы
double levelDb(int value, int length) {
   if (value == 0) {
      return corrDbFloor;
   }
   return std::max(corrDbFloor, 20.0 * std::log10(std::fabs(static_cast<double> (value))
                                                  / static_cast<double> (length)));
}

// Ряд τ = −corrSpanChips…+corrSpanChips: значения продолжаются периодически с периодом N
std::vector<double> periodicLevels(const std::vector<int>& values, int length) {
   std::vector<double> levels;

   levels.reserve(static_cast<std::size_t> (2 * corrSpanChips + 1));

   for (int tau = -corrSpanChips; tau <= corrSpanChips; ++tau) {
      const int index = ((tau % length) + length) % length;

      levels.push_back(levelDb(values[static_cast<std::size_t> (index)], length));
   }
   return levels;
}

// Шаг бинирования — ближайшее нечётное не меньше ⌈длина ряда / предел точек⌉: центральный бин
// центрируется ровно на τ = 0
int decimationStepOf(int length, int limit) {
   int step = (length + limit - 1) / limit;

   if ((step % 2) == 0) {
      ++step;
   }
   return step;
}

// Симметричное бинирование с сохранением границ значений: центральный бин [−h; +h], далее по
// step чипов в обе стороны; крайние бины неполные
std::vector<CorrBin> binSymmetric(const std::vector<double>& levels, int step) {
   const int half = (step - 1) / 2;
   std::vector<std::pair<int, int> > ranges;

   ranges.emplace_back(-half, half);

   for (int from = half + 1; from <= corrSpanChips; from += step) {
      const int to = std::min(corrSpanChips, from + step - 1);

      ranges.emplace_back(from, to);
      ranges.emplace_back(-to,  -from);
   }
   std::sort(ranges.begin(), ranges.end());
   std::vector<CorrBin> bins;

   bins.reserve(ranges.size());

   for (const std::pair<int, int>& range : ranges) {
      CorrBin bin;
      double  sum = 0.0;

      bin.lowDb  = levels[static_cast<std::size_t> (range.first + corrSpanChips)];
      bin.highDb = bin.lowDb;

      for (int tau = range.first; tau <= range.second; ++tau) {
         const double level = levels[static_cast<std::size_t> (tau + corrSpanChips)];

         bin.lowDb  = std::min(bin.lowDb, level);
         bin.highDb = std::max(bin.highDb, level);
         sum       += level;
      }
      const int count = range.second - range.first + 1;

      bin.averageDb = sum / static_cast<double> (count);
      bin.tauChips  = static_cast<double> (range.first + count / 2);
      bins.push_back(bin);
   }
   return bins;
}

// Таблицы ДК блока А_L1OC (А_L1OC.4(1)): кадр наблюдает тот же код, который излучает тракт
struct CodeWords {
   std::vector<std::uint64_t> packed;
   std::vector<std::uint64_t> doubled;
};

struct SatelliteCodes {
   CodeWords codeD;
   CodeWords codeP;
};

SatelliteCodes codesOf(int satellite) {
   glonass::RangingCodeL1OC code;

   code.initCodeTablesL1OC(satellite);
   SatelliteCodes out;

   out.codeD.packed  = packCode(code.codeTableD().data(), glonass::codeLengthD, 1);
   out.codeD.doubled = packCode(code.codeTableD().data(), glonass::codeLengthD, 2);
   out.codeP.packed  = packCode(code.codeTableP().data(), glonass::codeLengthP, 1);
   out.codeP.doubled = packCode(code.codeTableP().data(), glonass::codeLengthP, 2);
   return out;
}

// Границы оси уровней: низ — по наименьшему значению рядов, верх задаётся кадром
void applyAxis(const std::vector<CorrBin>& first, const std::vector<CorrBin>& second,
               bool acf, double& low, double& high) {
   double lowest  = first.front().lowDb;
   double highest = first.front().highDb;

   for (const std::vector<CorrBin>* bins : { &first, &second }) {
      for (const CorrBin& bin : *bins) {
         lowest  = std::min(lowest, bin.lowDb);
         highest = std::max(highest, bin.highDb);
      }
   }
   low  = std::floor(lowest / corrAxisStepDb) * corrAxisStepDb;
   high = acf ? corrAcfAxisHighDb : (std::ceil(highest / corrAxisStepDb) * corrAxisStepDb);
}

// ПАКФ одной компоненты: ряд кадра и показатели боковых лепестков
AcfSeries acfSeriesOf(const CodeWords& code, int length, int step) {
   const std::vector<int> values = correlationVector(code.packed, code.doubled, length);
   AcfSeries series;

   series.lengthChips = length;
   series.mainLobe    = values.front();
   std::map<int, int> sidelobes;

   for (std::size_t tau = 1; tau < values.size(); ++tau) {
      sidelobes[values[tau]] += 1;
      series.peakSidelobe     = std::max(series.peakSidelobe, std::abs(values[tau]));
   }

   for (const std::pair<const int, int>& item : sidelobes) {
      series.sidelobeValues.push_back(item.first);
      series.sidelobeCounts.push_back(item.second);
   }
   series.peakSidelobeDb = levelDb(series.peakSidelobe, length);
   series.bins           = binSymmetric(periodicLevels(values, length), step);
   return series;
}

// Огибающая ПВКФ по составу: env(τ) = max |R_ab(τ)| по парам a < b
CcfSeries ccfSeriesOf(const std::vector<CodeWords>& codes, int length, int step) {
   std::vector<int> envelope(static_cast<std::size_t> (length), 0);

   for (std::size_t a = 0; a < codes.size(); ++a) {
      for (std::size_t b = a + 1U; b < codes.size(); ++b) {
         for (int tau = 0; tau < length; ++tau) {
            const int value = std::abs(correlationAt(codes[a].packed, codes[b].doubled, tau,
                                                     length));

            envelope[static_cast<std::size_t> (tau)] =
               std::max(envelope[static_cast<std::size_t> (tau)], value);
         }
      }
   }
   CcfSeries series;

   series.lengthChips = length;
   series.peak        = *std::max_element(envelope.begin(), envelope.end());
   series.lowest      = *std::min_element(envelope.begin(), envelope.end());
   std::map<int, int> levels;

   for (const int value : envelope) {
      levels[value] += 1;
   }
   series.levelCount   = static_cast<int> (levels.size());
   series.shiftsAtPeak = levels[series.peak];
   series.peakDb       = levelDb(series.peak,   length);
   series.lowestDb     = levelDb(series.lowest, length);
   series.bins         = binSymmetric(periodicLevels(envelope, length), step);
   return series;
}

// Общая часть шапки: параметры прогона, от которых зависят дальномерные коды
std::string signalObject(const StreamRequest& request) {
   JsonObject signal;

   signal.addInt("sampleRateHz",    request.sampleRate);
   signal.addInt("referenceFreqHz", request.referenceFreq);
   signal.addInt("startSample",     request.startSample);
   signal.addRaw("satellites",    jsonIntegerArray(request.satellites));
   signal.addRaw("amplitudes",    jsonNumberArray(request.amplitudes, "%g"));
   signal.addRaw("initialPhases", jsonNumberArray(request.initialPhases, "%g"));
   return signal.str();
}

// Правила отображения: границы осей и параметры прореживания
std::string renderObject(double lowDb, double highDb, int step, std::size_t points,
                         const char* yLabel) {
   JsonObject decimation;

   decimation.addString("rule", "binMinMaxSymmetric");
   decimation.addInt("limit",  corrPointLimit);
   decimation.addInt("step",   step);
   decimation.addInt("points", static_cast<std::int64_t> (points));
   JsonObject render;

   render.addString("xLabel", "Сдвиг τ");
   render.addString("xUnit",  "чипы");
   render.addInt("xLowChips",  -corrSpanChips);
   render.addInt("xHighChips", corrSpanChips);
   render.addDouble("xStepChips", corrAxisStepChips);
   render.addString("yLabel", yLabel);
   render.addString("yUnit",  "дБ");
   render.addDouble("yLow",    lowDb);
   render.addDouble("yHigh",   highDb);
   render.addDouble("yStep",   corrAxisStepDb);
   render.addDouble("dbFloor", corrDbFloor);
   render.addRaw("decimation", decimation.str());
   return render.str();
}

// Ряды бинов одной компоненты
std::string binSeries(const std::vector<CorrBin>& bins) {
   std::vector<double> tau;
   std::vector<double> low;
   std::vector<double> high;
   std::vector<double> average;

   tau.reserve(bins.size());
   low.reserve(bins.size());
   high.reserve(bins.size());
   average.reserve(bins.size());

   for (const CorrBin& bin : bins) {
      tau.push_back(bin.tauChips);
      low.push_back(bin.lowDb);
      high.push_back(bin.highDb);
      average.push_back(bin.averageDb);
   }
   JsonObject series;

   series.addRaw("tauChips", jsonNumberArray(tau, "%.0f"));
   series.addRaw("minDb",    jsonNumberArray(low,     "%.3f"));
   series.addRaw("maxDb",    jsonNumberArray(high,    "%.3f"));
   series.addRaw("avgDb",    jsonNumberArray(average, "%.3f"));
   return series.str();
}

// Верхняя огибающая ряда — кривая кадра
std::vector<std::pair<double, double> > upperEnvelope(const std::vector<CorrBin>& bins) {
   std::vector<std::pair<double, double> > points;

   points.reserve(bins.size());

   for (const CorrBin& bin : bins) {
      points.emplace_back(bin.tauChips, bin.highDb);
   }
   return points;
}

// Полоса значений в бинах — размах ряда внутри бина
std::vector<Bin> spreadBand(const std::vector<CorrBin>& bins) {
   std::vector<Bin> band;

   band.reserve(bins.size());

   for (const CorrBin& bin : bins) {
      band.push_back(Bin{ bin.tauChips, bin.lowDb, bin.highDb });
   }
   return band;
}

// Перечисление значений боковых лепестков с долями; при большом наборе — только их число
std::string sidelobeText(const AcfSeries& series) {
   if (series.sidelobeValues.size() > sidelobeListLimit) {
      return "различных значений " + std::to_string(series.sidelobeValues.size());
   }
   const double total = static_cast<double> (series.lengthChips - 1);
   std::string  out;

   for (std::size_t i = 0; i < series.sidelobeValues.size(); ++i) {
      if (i > 0) {
         out += ", ";
      }
      out += numberRu(series.sidelobeValues[i]) + " ("
             + numberRu(static_cast<double> (series.sidelobeCounts[i]) / total * 100.0, 1) + " %)";
   }
   return out;
}
} // namespace

AcfFrame computeAcfFrame(const StreamRequest& request) {
   AcfFrame frame;

   // Кадр показывает один код: параметрами кадра строка запроса не управляет, номер НКА берётся
   // первым из состава прогона
   frame.satellite = request.satellites.front();
   const SatelliteCodes codes = codesOf(frame.satellite);

   frame.decimationStep = decimationStepOf(2 * corrSpanChips + 1, corrPointLimit);
   frame.codeD          = acfSeriesOf(codes.codeD, glonass::codeLengthD, frame.decimationStep);
   frame.codeP          = acfSeriesOf(codes.codeP, glonass::codeLengthP, frame.decimationStep);
   applyAxis(frame.codeD.bins, frame.codeP.bins, true, frame.axisLowDb, frame.axisHighDb);
   return frame;
}

CcfFrame computeCcfFrame(const StreamRequest& request) {
   if (request.satellites.size() < 2U) {
      throw glonass_params::ParamError(glonass_params::RejectKind::unrealizable, "j",
                                       "j: взаимнокорреляционная функция ансамбля не определена: "
                                       "в составе один НКА, пар нет");
   }
   std::vector<CodeWords> codesD;
   std::vector<CodeWords> codesP;

   codesD.reserve(request.satellites.size());
   codesP.reserve(request.satellites.size());

   for (const int satellite : request.satellites) {
      SatelliteCodes codes = codesOf(satellite);

      codesD.push_back(std::move(codes.codeD));
      codesP.push_back(std::move(codes.codeP));
   }
   CcfFrame  frame;
   const int count = static_cast<int> (request.satellites.size());

   frame.pairCount      = count * (count - 1) / 2;
   frame.decimationStep = decimationStepOf(2 * corrSpanChips + 1, corrPointLimit);
   frame.codeD          = ccfSeriesOf(codesD, glonass::codeLengthD, frame.decimationStep);
   frame.codeP          = ccfSeriesOf(codesP, glonass::codeLengthP, frame.decimationStep);
   applyAxis(frame.codeD.bins, frame.codeP.bins, false, frame.axisLowDb, frame.axisHighDb);
   return frame;
}

std::string acfFrameJson(const AcfFrame& frame, const StreamRequest& request) {
   JsonObject codeD;
   JsonObject codeP;
   const AcfSeries* series[2] = { &frame.codeD, &frame.codeP };
   JsonObject* target[2]      = { &codeD, &codeP };
   const char* family[2]      = { "коды Голда", "усечённые последовательности Касами" };
   const char* clause[2]      = { "2.2.1", "2.2.2" };

   for (int index = 0; index < 2; ++index) {
      target[index]->addString("family", family[index]);
      target[index]->addString("icdClause", clause[index]);
      target[index]->addInt("lengthChips",  series[index]->lengthChips);
      target[index]->addInt("mainLobe",     series[index]->mainLobe);
      target[index]->addInt("peakSidelobe", series[index]->peakSidelobe);
      target[index]->addDouble("peakSidelobeDb", series[index]->peakSidelobeDb);
      target[index]->addRaw("sidelobeValues", jsonIntegerArray(series[index]->sidelobeValues));
      target[index]->addRaw("sidelobeCounts", jsonIntegerArray(series[index]->sidelobeCounts));
   }
   JsonObject correlation;

   correlation.addInt("satellite", frame.satellite);
   correlation.addString("definition", "20·lg|R(τ)/N|");
   correlation.addString("source",     "таблицы ДК блока А_L1OC");
   correlation.addInt("spanChips", corrSpanChips);
   correlation.addRaw("codeD", codeD.str());
   correlation.addRaw("codeP", codeP.str());
   JsonObject series2;

   series2.addRaw("codeD", binSeries(frame.codeD.bins));
   series2.addRaw("codeP", binSeries(frame.codeP.bins));
   JsonObject root;

   root.addString("kind",  "acf");
   root.addString("title", "Периодическая автокорреляционная функция дальномерного кода");
   root.addRaw("signal",      signalObject(request));
   root.addRaw("correlation", correlation.str());
   root.addRaw("render",      renderObject(frame.axisLowDb, frame.axisHighDb,
                                           frame.decimationStep, frame.codeD.bins.size(),
                                           "Уровень, дБ отн. главного лепестка"));
   root.addRaw("series",      series2.str());
   return root.str();
}

std::string ccfFrameJson(const CcfFrame& frame, const StreamRequest& request) {
   JsonObject codeD;
   JsonObject codeP;
   const CcfSeries* series[2] = { &frame.codeD, &frame.codeP };
   JsonObject* target[2]      = { &codeD, &codeP };
   const char* family[2]      = { "коды Голда", "усечённые последовательности Касами" };
   const char* clause[2]      = { "2.2.1", "2.2.2" };

   for (int index = 0; index < 2; ++index) {
      target[index]->addString("family", family[index]);
      target[index]->addString("icdClause", clause[index]);
      target[index]->addInt("lengthChips", series[index]->lengthChips);
      target[index]->addInt("peak",        series[index]->peak);
      target[index]->addDouble("peakDb", series[index]->peakDb);
      target[index]->addInt("shiftsAtPeak", series[index]->shiftsAtPeak);
      target[index]->addInt("levelCount",   series[index]->levelCount);
      target[index]->addInt("lowest",       series[index]->lowest);
      target[index]->addDouble("lowestDb", series[index]->lowestDb);
   }
   JsonObject ensemble;

   ensemble.addInt("satelliteCount", static_cast<std::int64_t> (request.satellites.size()));
   ensemble.addInt("pairCount",      frame.pairCount);
   ensemble.addString("definition", "env(τ) = max |R_ab(τ)| по парам a < b состава J");
   ensemble.addString("level",      "20·lg|env(τ)/N|");
   ensemble.addString("source",     "таблицы ДК блока А_L1OC");
   ensemble.addInt("spanChips", corrSpanChips);
   ensemble.addRaw("codeD", codeD.str());
   ensemble.addRaw("codeP", codeP.str());
   JsonObject series2;

   series2.addRaw("codeD", binSeries(frame.codeD.bins));
   series2.addRaw("codeP", binSeries(frame.codeP.bins));
   JsonObject root;

   root.addString("kind",  "ccf");
   root.addString("title", "Огибающая периодической взаимнокорреляционной функции ансамбля");
   root.addRaw("signal",   signalObject(request));
   root.addRaw("ensemble", ensemble.str());
   root.addRaw("render",   renderObject(frame.axisLowDb, frame.axisHighDb, frame.decimationStep,
                                        frame.codeD.bins.size(),
                                        "Уровень, дБ отн. длины кода"));
   root.addRaw("series",   series2.str());
   return root.str();
}

std::string acfFrameSvg(const AcfFrame& frame, const StreamRequest& request) {
   const std::string codes = "НКА j = " + std::to_string(frame.satellite)
                             + " · ДК_L1OCd: коды Голда, N_d = "
                             + numberRu(frame.codeD.lengthChips) + " чипов"
                             + " · ДК_L1OCp: усечённые последовательности Касами, N_p = "
                             + numberRu(frame.codeP.lengthChips) + " чипов";
   const std::string levels = "Уровень 20·lg|R(τ)/N| · верхняя огибающая · боковые значения "
                              "ДК_L1OCd: " + sidelobeText(frame.codeD) + " · ДК_L1OCp: "
                              + sidelobeText(frame.codeP);
   SvgCanvas canvas("Периодическая автокорреляционная функция дальномерного кода",
                    { codes, levels }, "Сдвиг τ, чипы", "Уровень, дБ отн. главного лепестка",
                    "Периодическая автокорреляционная функция дальномерных кодов L1OC состава "
                    + compositionOf(request.satellites)
                    + ". signal-service-l1oc, ИКД ГЛОНАСС L1OC ред. 1.0 (2016).");

   canvas.setX(-corrSpanChips, corrSpanChips, corrAxisStepChips, 5);
   canvas.setY(frame.axisLowDb, frame.axisHighDb, corrAxisStepDb, 5);
   canvas.polyline(upperEnvelope(frame.codeP.bins), svg::series2, 1.3);
   canvas.polyline(upperEnvelope(frame.codeD.bins), svg::series1, 1.5);
   const std::string mark = "максимум бокового лепестка ДК_L1OCd: "
                            + numberRu(frame.codeD.peakSidelobeDb, 3) + " дБ";

   canvas.horizontalLine(frame.codeD.peakSidelobeDb, svg::markColor, svg::dashMark, mark);
   canvas.legend({ LegendRow{ svg::series1, svg::dashSolid,
                              "ДК_L1OCd (N = " + numberRu(frame.codeD.lengthChips) + ")" },
                   LegendRow{ svg::series2, svg::dashSolid,
                              "ДК_L1OCp (N = " + numberRu(frame.codeP.lengthChips) + ")" },
                   LegendRow{ svg::markColor, svg::dashMark,
                              "Опорный уровень " + numberRu(frame.codeD.peakSidelobeDb, 3)
                              + " дБ" } }, LegendCorner::rightBottom);
   return canvas.str();
}

std::string ccfFrameSvg(const CcfFrame& frame, const StreamRequest& request) {
   const std::string ensemble = compositionOf(request.satellites) + " · пар "
                                + numberRu(frame.pairCount)
                                + " · огибающая = максимум |R(τ)| по парам состава";
   const std::string peaks = "Максимум ДК_L1OCd: " + numberRu(frame.codeD.peak) + "/"
                             + numberRu(frame.codeD.lengthChips) + " = "
                             + numberRu(frame.codeD.peakDb, 3) + " дБ · ДК_L1OCp: "
                             + numberRu(frame.codeP.peak) + "/"
                             + numberRu(frame.codeP.lengthChips) + " = "
                             + numberRu(frame.codeP.peakDb, 3) + " дБ";
   SvgCanvas canvas("Огибающая периодической взаимнокорреляционной функции ансамбля",
                    { ensemble, peaks }, "Сдвиг τ, чипы", "Уровень, дБ отн. длины кода",
                    "Огибающая периодической взаимнокорреляционной функции ансамбля дальномерных "
                    "кодов L1OC по составу активных НКА. signal-service-l1oc, "
                    "ИКД ГЛОНАСС L1OC ред. 1.0 (2016).");

   canvas.setX(-corrSpanChips, corrSpanChips, corrAxisStepChips, 5);
   canvas.setY(frame.axisLowDb, frame.axisHighDb, corrAxisStepDb, 5);
   canvas.envelope(spreadBand(frame.codeP.bins), svg::series2, 0.16);
   canvas.polyline(upperEnvelope(frame.codeP.bins), svg::series2, 1.3);
   canvas.envelope(spreadBand(frame.codeD.bins), svg::series1, 0.20);
   canvas.polyline(upperEnvelope(frame.codeD.bins), svg::series1, 1.5);
   const CcfSeries* series[2] = { &frame.codeD, &frame.codeP };
   const char* name[2]        = { "ДК_L1OCd", "ДК_L1OCp" };
   const char* color[2]       = { svg::series1, svg::series2 };
   std::vector<LegendRow> rows;

   for (int index = 0; index < 2; ++index) {
      rows.push_back(LegendRow{ color[index], svg::dashSolid,
                                std::string(name[index]) + " (N = "
                                + numberRu(series[index]->lengthChips) + "): уровней "
                                + numberRu(series[index]->levelCount) + " · сдвигов на максимуме "
                                + numberRu(series[index]->shiftsAtPeak) + " из "
                                + numberRu(series[index]->lengthChips) });
   }
   canvas.legend(rows, LegendCorner::rightBottom);
   return canvas.str();
}
} // namespace glonass_service
