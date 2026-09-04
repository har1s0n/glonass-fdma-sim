#include "frame_navline.h"

#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "glonass/source_config_l1oc.h"
#include "glonass/types.h"
#include "json_writer.h"
#include "request_params_l1oc.h"
#include "state_metrics.h"
#include "svg_canvas.h"

namespace glonass_service {
namespace {
constexpr const char* keyStartSample = "n0";

const char*lineTypeText(glonass::LineTypeL1OC lineType) {
   switch (lineType) {
     case glonass::LineTypeL1OC::anomalous1: return "строка 1-го типа";

     case glonass::LineTypeL1OC::anomalous2: return "строка 2-го типа";

     default:                                return "нормальная строка";
   }
}

std::string scenarioLine(const StreamRequest& request) {
   std::string line = compositionOf(request.satellites);

   line += " · Fs = " + numberRu(static_cast<double> (request.sampleRate) / 1.0e6, 1) + " МГц";
   line += " · f₀ = " + numberRu(static_cast<double> (request.referenceFreq) / 1.0e6, 3) + " МГц";
   line += " · n₀ = " + numberRu(static_cast<double> (request.startSample));
   line += " · t = n₀/Fs = " + numberRu(static_cast<double> (request.startSample)
                                        / static_cast<double> (request.sampleRate), 3) + " с";
   return line;
}

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

// Зоны полей строки в блоке render: полуинтервал по оси символов СК и длина поля в битах
std::string fieldArray(const std::vector<NavLineField>& fields) {
   std::string out = "[";

   for (std::size_t i = 0; i < fields.size(); ++i) {
      JsonObject field;

      field.addString("name", fields[i].name);
      field.addDouble("from", fields[i].from);
      field.addDouble("to",   fields[i].to);
      field.addInt("bits", fields[i].bits);

      if (i > 0) {
         out += ", ";
      }
      out += field.str();
   }
   return out + "]";
}

// Состояние регистра СК S[1…6] массивом (Б_L1OC.5)
std::string convStateArray(const glonass::ConvStateL1OC& state) {
   std::vector<int> values;

   for (const glonass::Bit bit : state) {
      values.push_back(static_cast<int> (bit));
   }
   return jsonIntegerArray(values);
}
} // namespace

NavLineFrame computeNavLineFrame(const StreamRequest& request) {
   if (request.startSample
       > (std::numeric_limits<std::int64_t>::max() / glonass::symbolRateL1OC)) {
      throw glonass_params::ParamError(glonass_params::RejectKind::badValue, keyStartSample,
                                       "n0: произведение n₀·R_с вне разрядности счётчика символов");
   }
   const glonass::SourceConfigL1OC config                = sourceConfigOf(request);
   const glonass::PayloadProviderL1OC& payloadOfLineL1OC =
      config.satellites.front().payloadOfLineL1OC;

   // Координаты строки — из блока Б_L1OC (Б_L1OC.9, InitMessageAtSampleL1OC): формулы
   // Б_L1OC.4(4) не воспроизводятся в кадре повторно
   glonass::NavMessageL1OC message;

   message.initMessageAtSampleL1OC(config.globalStartSample, config.sampleRate,
                                   payloadOfLineL1OC);
   NavLineFrame frame;

   frame.lineIndex       = message.lineIndex();
   frame.convSymbolIndex = message.convSymbolIndex();
   frame.symbolPhase     = static_cast<double> (message.symbolPhaseAccumulator())
                           / static_cast<double> (request.sampleRate);

   const glonass::LineContentL1OC content = payloadOfLineL1OC(frame.lineIndex);
   const glonass::BuiltLineL1OC   built   = glonass::buildLineL1OC(content,
                                                                   glonass::ConvStateL1OC{});

   frame.lineType     = content.lineType;
   frame.infoBits     = glonass::lineInfoBits(content.lineType);
   frame.lineBitCount = glonass::lineBits(content.lineType);
   frame.lineLength   = built.lineLength;
   frame.convStateOut = built.convStateOut;
   frame.lineSymbols.reserve(static_cast<std::size_t> (frame.lineLength));

   for (int index = 0; index < frame.lineLength; ++index) {
      frame.lineSymbols.push_back(static_cast<int> (
                                     built.lineSymbols[static_cast<std::size_t> (index)]));

      if (frame.lineSymbols.back() == 1) {
         ++frame.onesCount;
      }

      if ((index > 0)
          && (frame.lineSymbols[static_cast<std::size_t> (index)]
              != frame.lineSymbols[static_cast<std::size_t> (index - 1)])) {
         ++frame.transitions;
      }
   }

   // Зоны полей строки: бит t информационного блока даёт символы 2t и 2t+1 (Б_L1OC.6);
   // состав строки — СМВ ‖ ЦИ ‖ ЦК ([ИКД-L1OC] 4.2.2.1, 4.4)
   const int parityBits = frame.lineBitCount - navlineSmvBits - frame.infoBits;

   frame.fields.push_back(NavLineField{ 0.0, 2.0 * navlineSmvBits, navlineSmvBits, "СМВ" });
   frame.fields.push_back(NavLineField{ 2.0 * navlineSmvBits,
                                        2.0 * (navlineSmvBits + frame.infoBits),
                                        frame.infoBits, "ЦИ" });
   frame.fields.push_back(NavLineField{ 2.0 * (navlineSmvBits + frame.infoBits),
                                        2.0 * frame.lineBitCount, parityBits, "ЦК" });

   frame.symbolDurationMs = 1.0e3 / static_cast<double> (glonass::symbolRateL1OC);
   frame.lineDurationS    = static_cast<double> (frame.lineLength)
                            / static_cast<double> (glonass::symbolRateL1OC);
   return frame;
}

std::string navLineFrameJson(const NavLineFrame& frame, const StreamRequest& request) {
   JsonObject line;

   line.addInt("lineIndex", frame.lineIndex);
   line.addString("lineType", lineTypeName(frame.lineType));
   line.addInt("smvBits",      navlineSmvBits);
   line.addInt("infoBits",     frame.infoBits);
   line.addInt("lineBits",     frame.lineBitCount);
   line.addInt("lineLength",   frame.lineLength);
   line.addInt("symbolRateHz", glonass::symbolRateL1OC);
   line.addDouble("symbolDurationMs", frame.symbolDurationMs);
   line.addDouble("durationS",        frame.lineDurationS);
   line.addInt("convSymbolIndex", frame.convSymbolIndex);
   line.addDouble("symbolPhase", frame.symbolPhase);
   line.addRaw("convStateOut", convStateArray(frame.convStateOut));
   line.addInt("onesCount",   frame.onesCount);
   line.addInt("transitions", frame.transitions);
   line.addString("source",
                  "прогон блока Б_L1OC: слой содержания прогона, состояние регистра СК при "
                  "запуске нулевое");
   line.addString("icdClause", "4.2.2.1, 4.4");
   JsonObject render;

   render.addString("xLabel", "Символ свёрточного кода в строке");
   render.addString("xUnit",  "символ");
   render.addDouble("xLow",  0.0);
   render.addDouble("xHigh", static_cast<double> (frame.lineLength));
   render.addDouble("xStep", navlineAxisStepSymbols);
   render.addString("yLabel", "Символ b_line в алфавите {−1, +1}");
   render.addDouble("yLow",  -navlineAxisLimit);
   render.addDouble("yHigh", navlineAxisLimit);
   render.addDouble("yStep", navlineAxisStep);
   render.addString("levelRule",  "уровень 1 − 2·b_line");
   render.addString("markerRule", "вертикаль на символе w[n₀]");
   render.addInt("marker", frame.convSymbolIndex);
   render.addRaw("fields", fieldArray(frame.fields));
   JsonObject series;

   series.addRaw("symbol", jsonIntegerArray(frame.lineSymbols));
   JsonObject root;

   root.addString("kind",  "navline");
   root.addString("title", "Строка навигационного сообщения: символы свёрточного кода");
   root.addRaw("signal", signalObject(request));
   root.addRaw("line",   line.str());
   root.addRaw("render", render.str());
   root.addRaw("series", series.str());
   return root.str();
}

std::string navLineFrameSvg(const NavLineFrame& frame, const StreamRequest& request) {
   const std::string marker = "w[n₀] = " + numberRu(frame.convSymbolIndex);
   std::string line         = "Строка ℓ = " + numberRu(static_cast<double> (frame.lineIndex))
                              + " · " + lineTypeText(frame.lineType)
                              + " · n_с = " + numberRu(frame.lineBitCount) + " бит"
                              + " · L_с = " + numberRu(frame.lineLength) + " символов"
                              + " · символ СК " + numberRu(frame.symbolDurationMs, 1) + " мс"
                              + " · строка " + numberRu(frame.lineDurationS, 1) + " с"
                              + " · " + marker
                              + " · фаза символа " + numberRu(frame.symbolPhase, 3);
   SvgCanvas canvas("Строка навигационного сообщения: символы свёрточного кода",
                    { scenarioLine(request), line }, "Символ свёрточного кода в строке",
                    "Символ b_line в алфавите {−1, +1}",
                    "Строка навигационного сообщения L1OC на выходе свёрточного кода (133,171): "
                    "поля СМВ, ЦИ и проверочные биты циклического кода, отметка текущего символа. "
                    "signal-service-l1oc, ИКД ГЛОНАСС L1OC ред. 1.0 (2016).");

   canvas.setX(0.0, static_cast<double> (frame.lineLength), navlineAxisStepSymbols, 5);
   canvas.setY(-navlineAxisLimit, navlineAxisLimit, navlineAxisStep, 5, 1);

   // Поля строки: заливкой выделены СМВ и ЦК — границы ЦИ читаются как промежуток между ними
   const char* fieldColor[3] = { svg::series3, nullptr, svg::series2 };

   for (std::size_t index = 0; index < frame.fields.size(); ++index) {
      const NavLineField& field = frame.fields[index];
      const char* color         = (index < 3) ? fieldColor[index] : nullptr;

      if (color != nullptr) {
         canvas.band(field.from, field.to, color, 0.12);
      }
      canvas.labelAt(0.5 * (field.from + field.to), navlineFieldLabelLevel, field.name, 11,
                     svg::textSecond);
   }

   // Ряд символов: символ w занимает полуинтервал [w; w+1), последний доводится до конца строки
   std::vector<std::pair<double, double> > points;

   points.reserve(frame.lineSymbols.size() + 1);

   for (std::size_t index = 0; index < frame.lineSymbols.size(); ++index) {
      points.emplace_back(static_cast<double> (index),
                          1.0 - 2.0 * static_cast<double> (frame.lineSymbols[index]));
   }

   if (!points.empty()) {
      points.emplace_back(static_cast<double> (frame.lineLength), points.back().second);
   }
   canvas.steps(points, svg::series1, 1.4);
   canvas.verticalLine(static_cast<double> (frame.convSymbolIndex), svg::markColor, svg::dashMark,
                       marker, true, frame.convSymbolIndex > (frame.lineLength / 2));
   canvas.legend({ LegendRow{ svg::series1, svg::dashSolid, "Символ b_line = 1 − 2·b" },
                   LegendRow{ svg::series3, svg::dashFill, "Поле СМВ ("
                              + numberRu(navlineSmvBits) + " бит)" },
                   LegendRow{ svg::series2, svg::dashFill, "Проверочные биты ЦК ("
                              + numberRu(frame.lineBitCount - navlineSmvBits - frame.infoBits)
                              + " бит)" } },
                 LegendCorner::rightBottom);
   return canvas.str();
}
} // namespace glonass_service
