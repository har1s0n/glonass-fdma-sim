#include "fft_radix2.h"
#include "frame_correlation.h"
#include "frame_level.h"
#include "frame_navline.h"
#include "frame_psd.h"
#include "frame_waveform.h"
#include "svg_canvas.h"

#include "glonass/nav_message_l1oc.h"
#include "glonass/ranging_code_l1oc.h"
#include "glonass/types.h"
#include "request_params_l1oc.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

// Кадры микросервиса L1OC — базовый слой, кадры спектральной плотности мощности,
// осциллограммы квадратур, гистограммы мгновенных значений и корреляционных функций
// дальномерных кодов

namespace {
using glonass_service::AcfFrame;
using glonass_service::acfFrameJson;
using glonass_service::acfFrameSvg;
using glonass_service::CcfFrame;
using glonass_service::ccfFrameJson;
using glonass_service::ccfFrameSvg;
using glonass_service::computeAcfFrame;
using glonass_service::computeCcfFrame;
using glonass_service::computeLevelFrame;
using glonass_service::computeNavLineFrame;
using glonass_service::computePsdFrame;
using glonass_service::computeWaveformFrame;
using glonass_service::dftDirect;
using glonass_service::fftRadix2;
using glonass_service::LevelFrame;
using glonass_service::levelFrameJson;
using glonass_service::levelFrameSvg;
using glonass_service::NavLineFrame;
using glonass_service::navLineFrameJson;
using glonass_service::navLineFrameSvg;
using glonass_service::numberRu;
using glonass_service::PsdFrame;
using glonass_service::psdFrameJson;
using glonass_service::psdFrameSvg;
using glonass_service::StreamRequest;
using glonass_service::WaveformFrame;
using glonass_service::waveformFrameJson;
using glonass_service::waveformFrameSvg;

// Символы дальномерного кода блока А_L1OC в алфавите {+1, −1}: c -> 1 − 2c
std::vector<int> codeSymbols(int satellite, bool pilot) {
   glonass::RangingCodeL1OC code;

   code.initCodeTablesL1OC(satellite);
   std::vector<int> symbols;

   if (pilot) {
      for (const glonass::Bit bit : code.codeTableP()) {
         symbols.push_back(1 - 2 * static_cast<int> (bit));
      }
   } else {
      for (const glonass::Bit bit : code.codeTableD()) {
         symbols.push_back(1 - 2 * static_cast<int> (bit));
      }
   }
   return symbols;
}

// Прямое суммирование корреляции — независимая проверка счёта через XOR и popcount
int correlationDirect(const std::vector<int>& a, const std::vector<int>& b, int tau) {
   const int length = static_cast<int> (a.size());
   int sum          = 0;

   for (int i = 0; i < length; ++i) {
      sum += a[static_cast<std::size_t> (i)]
             * b[static_cast<std::size_t> ((i + tau) % length)];
   }
   return sum;
}

bool contains(const std::string& text, const std::string& fragment) {
   return text.find(fragment) != std::string::npos;
}

std::vector<int> range(int first, int last) {
   std::vector<int> values;

   for (int value = first; value <= last; ++value) {
      values.push_back(value);
   }
   return values;
}

// Конфигурация прогона: J = {first…last}, A_j = 1, φ_{0,j} = 0, f₀ = f_L1OC, n₀ = 0
StreamRequest configuration(int first, int last) {
   StreamRequest request;

   request.sampleRate    = 20000000;
   request.referenceFreq = glonass::carrierFreqL1OC;
   request.startSample   = 0;
   request.satellites    = range(first, last);
   request.amplitudes    = std::vector<double> (request.satellites.size(), 1.0);
   request.initialPhases = std::vector<double> (request.satellites.size(), 0.0);
   return request;
}

// Конфигурация с привязкой: J = {1…3}, заданные Fs и n₀
StreamRequest configurationAt(std::int64_t sampleRate, std::int64_t startSample) {
   StreamRequest request = configuration(1, 3);

   request.sampleRate  = sampleRate;
   request.startSample = startSample;
   return request;
}

// Символы строки текстом: сверка с независимым счётом посимвольно
std::string symbolText(const std::vector<int>& symbols, std::size_t from, std::size_t count) {
   std::string text;

   for (std::size_t i = from; (i < symbols.size()) && (i < from + count); ++i) {
      text += static_cast<char> ('0' + symbols[i]);
   }
   return text;
}

// Единиц среди символов полуинтервала [from; to)
int onesWithin(const std::vector<int>& symbols, double from, double to) {
   int count = 0;

   for (std::size_t i = static_cast<std::size_t> (from);
        (i < symbols.size()) && (static_cast<double> (i) < to); ++i) {
      count += symbols[i];
   }
   return count;
}

// Детерминированная последовательность для сверки преобразований
std::vector<std::complex<double> > testSignal(std::size_t length) {
   std::vector<std::complex<double> > data(length);

   for (std::size_t i = 0; i < length; ++i) {
      data[i] = std::complex<double> (std::cos(0.1 * static_cast<double> (i)),
                                      std::sin(0.07 * static_cast<double> (i)));
   }
   return data;
}
} // namespace

// ───────────────────── быстрое преобразование Фурье ─────────────────────

// Сверка с прямым дискретным преобразованием
TEST(FftRadix2L1OC, Test1_MatchesDirectTransform) {
   const std::vector<std::complex<double> > source   = testSignal(32);
   const std::vector<std::complex<double> > expected = dftDirect(source);
   std::vector<std::complex<double> > actual         = source;

   fftRadix2(actual);
   ASSERT_EQ(actual.size(), expected.size());
   double worst = 0.0;

   for (std::size_t i = 0; i < actual.size(); ++i) {
      worst = std::max(worst, std::abs(actual[i] - expected[i]));
   }
   EXPECT_LT(worst, 1e-12);
}

// Обратимость: обратное преобразование через сопряжение восстанавливает вход
TEST(FftRadix2L1OC, Test2_RoundTrip) {
   const std::vector<std::complex<double> > source = testSignal(256);
   std::vector<std::complex<double> > data         = source;

   fftRadix2(data);

   for (std::complex<double>& value : data) {
      value = std::conj(value);
   }
   fftRadix2(data);
   double worst = 0.0;

   for (std::size_t i = 0; i < data.size(); ++i) {
      worst = std::max(worst, std::abs(std::conj(data[i]) / 256.0 - source[i]));
   }
   EXPECT_LT(worst, 1e-12);
}

// Постоянная составляющая: единичный вход даёт N в нулевой корзине и нули в прочих
TEST(FftRadix2L1OC, Test3_ConstantInput) {
   std::vector<std::complex<double> > data(64, std::complex<double> (1.0, 0.0));

   fftRadix2(data);
   EXPECT_NEAR(data[0].real(), 64.0, 1e-12);

   for (std::size_t i = 1; i < data.size(); ++i) {
      EXPECT_LT(std::abs(data[i]), 1e-12) << "корзина " << i;
   }
}

// ───────────────────── числовой формат подписей (Р3.8) ─────────────────────

TEST(SvgCanvasL1OC, Test4_RussianNumberFormat) {
   EXPECT_EQ(numberRu(1600.995, 3), "1\xE2\x80\xAF" "600,995"); // узкий неразрывный пробел
   EXPECT_EQ(numberRu(-23.939, 3),  "\xE2\x88\x92" "23,939");   // знак минус U+2212
   EXPECT_EQ(numberRu(262144),      "262\xE2\x80\xAF" "144");
   EXPECT_EQ(numberRu(0.5, 1),      "0,5");
   EXPECT_EQ(numberRu(-10),         "\xE2\x88\x92" "10");
}

// ───────────────────── кадр спектральной плотности мощности ─────────────────────

// Параметры оценки и прореживания
TEST(FramePsdL1OC, Test5_EstimateParameters) {
   const PsdFrame frame = computePsdFrame(configuration(1, 3));

   EXPECT_EQ(frame.decimationStep, 5);
   EXPECT_EQ(frame.bins.size(),    1639U);
   EXPECT_NEAR(frame.resolutionHz,                2441.40625,       1e-9); // Fs / 8192
   EXPECT_NEAR(frame.binStepHz,                   5.0 * 2441.40625, 1e-6);

   // Границы оси кратны шагу 10 дБ и охватывают данные с запасом 5 дБ
   EXPECT_NEAR(std::fmod(frame.axisLowDb, 10.0),  0.0,              1e-9);
   EXPECT_NEAR(std::fmod(frame.axisHighDb, 10.0), 0.0,              1e-9);

   for (const auto& bin : frame.bins) {
      EXPECT_GE(bin.highDb,    bin.lowDb);
      EXPECT_GE(bin.averageDb, bin.lowDb);
      EXPECT_LE(bin.averageDb, bin.highDb);
      EXPECT_GT(bin.highDb, frame.axisLowDb);
   }
}

// Нормировка по сглаженной оценке: максимум кривой лежит около 0 дБ, а не смещён вниз
// разбросом усреднённой периодограммы
TEST(FramePsdL1OC, Test6_NormalisationNearZero) {
   const PsdFrame frame = computePsdFrame(configuration(1, 24));
   double top           = frame.bins.front().averageDb;

   for (const auto& bin : frame.bins) {
      top = std::max(top, bin.averageDb);
   }
   EXPECT_GT(top, -1.0);
   EXPECT_LT(top, 2.0);
}

// Контрольные значения независимого счёта (Python) для J = 1…24, n₀ = 0, Fs = 20 МГц
TEST(FramePsdL1OC, Test7_LevelsMatchIndependentComputation) {
   const PsdFrame frame = computePsdFrame(configuration(1, 24));

   ASSERT_EQ(frame.bins.size(), 1639U);
   struct Reference {
      std::size_t index;
      double      freqMHz;
      double      levelDb;
   };
   const Reference references[] = { {    0, -9.995, -28.246 },
      {  409, -5.002, -19.511 },
      {  819, +0.002, -3.117  },
      { 1229, +5.007, -19.902 },
      { 1638, +9.998, -28.140 } };

   for (const Reference& reference : references) {
      const auto& bin = frame.bins[reference.index];

      EXPECT_NEAR(bin.centerHz / 1.0e6, reference.freqMHz, 1e-3) << "бин " << reference.index;
      EXPECT_NEAR(bin.averageDb,        reference.levelDb, 0.01) << "бин " << reference.index;
   }
}

// Ряды и правила отображения выводятся в ответе: без них кадр невоспроизводим (Р3.17)
TEST(FramePsdL1OC, Test8_JsonCarriesSeriesAndRenderRules) {
   const StreamRequest request = configuration(1, 3);
   const std::string   json    = psdFrameJson(computePsdFrame(request), request);

   EXPECT_TRUE(contains(json, "\"kind\": \"psd\""));
   EXPECT_TRUE(contains(json, "\"segmentLength\": 8192"));
   EXPECT_TRUE(contains(json, "\"segmentCount\": 32"));
   EXPECT_TRUE(contains(json, "\"window\": \"hann\""));
   EXPECT_TRUE(contains(json, "\"rule\": \"binMinMax\""));
   EXPECT_TRUE(contains(json, "\"limit\": 2000"));
   EXPECT_TRUE(contains(json, "\"points\": 1639"));
   EXPECT_TRUE(contains(json, "\"centerHz\""));
   EXPECT_TRUE(contains(json, "\"minDb\""));
   EXPECT_TRUE(contains(json, "\"maxDb\""));
   EXPECT_TRUE(contains(json, "\"avgDb\""));
   EXPECT_FALSE(contains(json, "nan"));
   EXPECT_FALSE(contains(json, "inf"));
}

// Изображение строится по шаблону Р3: заголовок содержательный, идентификатора kind нет,
// строки происхождения нет, оси и легенда на месте
TEST(FramePsdL1OC, Test9_SvgFollowsFrameTemplate) {
   const StreamRequest request = configuration(1, 3);
   const std::string   image   = psdFrameSvg(computePsdFrame(request), request);

   EXPECT_TRUE(contains(image, "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 960 540\""));
   EXPECT_TRUE(contains(image, "<title>Спектральная плотность мощности суммарного сигнала L1OC</title>"));
   EXPECT_TRUE(contains(image, "clipPath id=\"plot\""));
   EXPECT_TRUE(contains(image, "Частота относительно f₀, МГц"));
   EXPECT_TRUE(contains(image, "СПМ, дБ отн. максимума"));
   EXPECT_TRUE(contains(image, "Полоса модели B_model = 2,046 МГц"));
   EXPECT_TRUE(contains(image, "</svg>"));

   // на изображении нет ни идентификатора kind, ни строки происхождения (решение 20.08.2026)
   EXPECT_FALSE(contains(image, ">Кадр psd"));
   EXPECT_FALSE(contains(image, "signal-service-l1oc 1.0 ·"));
   EXPECT_FALSE(contains(image, "sha256"));
   EXPECT_FALSE(contains(image, "nan"));
}

// Кадр не зависит от порядка обращений и воспроизводится побитно на тех же параметрах
TEST(FramePsdL1OC, Test10_Reproducible) {
   const StreamRequest request = configuration(1, 3);

   EXPECT_EQ(psdFrameSvg(computePsdFrame(request), request),
             psdFrameSvg(computePsdFrame(request), request));
}

// ───────────────────── кадр осциллограммы квадратур ─────────────────────

// Окно кадра: 16 чипов уплотнения при Fs = 20 МГц дают 313 отсчётов. Разметка компонент —
// по кодовой фазе (А_L1OC.5–А_L1OC.7); при n₀ = 0 границы чипов целые
TEST(FrameWaveformL1OC, Test11_WindowAndZones) {
   const WaveformFrame frame = computeWaveformFrame(configuration(5, 5));

   EXPECT_EQ(frame.sampleCount,       313);
   EXPECT_EQ(frame.chip.size(),       313U);
   EXPECT_EQ(frame.inphase.size(),    313U);
   EXPECT_EQ(frame.quadrature.size(), 313U);
   EXPECT_NEAR(frame.chipSamples, 19.550342130987293, 1e-12); // Fs / f_T1
   EXPECT_NEAR(frame.peakBound,   1.0,                1e-12); // η·ΣA_j при |J| = 1, A = 1
   EXPECT_NEAR(frame.axisLimit,   1.35,               1e-12);
   EXPECT_NEAR(frame.axisStep,    0.5,                1e-12);

   ASSERT_EQ(frame.zones.size(), 16U);

   for (std::size_t index = 0; index < frame.zones.size(); ++index) {
      EXPECT_NEAR(frame.zones[index].from, static_cast<double> (index),       1e-12);
      EXPECT_NEAR(frame.zones[index].to,   static_cast<double> (index) + 1.0, 1e-12);
      EXPECT_EQ(frame.zones[index].select, static_cast<int> (index % 2U)) << "чип " << index;
   }
   EXPECT_NEAR(frame.chip.front(), 0.0,                            1e-12);
   EXPECT_NEAR(frame.chip.back(),  312.0 * 1023000.0 / 20000000.0, 1e-12);
}

// Одиночный источник при Δf = 0 и φ_{0,j} = 0: η = 1, квадратура Q нулевая, |I| ≡ 1.
// Число переходов знака в окне — 16 (независимый счёт)
TEST(FrameWaveformL1OC, Test12_SingleSourceQuadratures) {
   const WaveformFrame frame = computeWaveformFrame(configuration(5, 5));
   int positive              = 0;
   int negative              = 0;
   int transitions           = 0;

   for (std::size_t index = 0; index < frame.inphase.size(); ++index) {
      EXPECT_EQ(frame.quadrature[index],         0.0) << "отсчёт " << index;
      EXPECT_EQ(std::fabs(frame.inphase[index]), 1.0) << "отсчёт " << index;

      if (frame.inphase[index] > 0.0) {
         ++positive;
      } else {
         ++negative;
      }

      if ((index > 0) && (frame.inphase[index] != frame.inphase[index - 1])) {
         ++transitions;
      }
   }
   EXPECT_EQ(positive,    136);
   EXPECT_EQ(negative,    177);
   EXPECT_EQ(transitions, 16);
}

// Контрольные отсчёты независимого счёта (Python) для j = 5, n₀ = 0, Fs = 20 МГц.
// Чипы компоненты L1OCd (σ = 0) держат знак; чипы L1OCp (σ = 1) делит меандр пополам
TEST(FrameWaveformL1OC, Test13_SamplesMatchIndependentComputation) {
   const WaveformFrame frame = computeWaveformFrame(configuration(5, 5));

   ASSERT_EQ(frame.inphase.size(), 313U);
   struct Reference {
      std::size_t index;
      double      inphase;
   };
   const Reference references[] = { {   0, -1.0 }, {  19, -1.0 }, {  20, +1.0 }, {  29, +1.0 },
      {  30, -1.0 }, {  39, -1.0 }, {  40, +1.0 }, {  58, +1.0 },
      {  59, +1.0 }, {  68, +1.0 }, {  69, -1.0 }, {  78, -1.0 },
      {  79, -1.0 }, {  97, -1.0 }, { 196, +1.0 }, { 215, +1.0 },
      { 216, +1.0 }, { 224, +1.0 }, { 225, -1.0 }, { 234, -1.0 },
      { 294, +1.0 }, { 303, +1.0 }, { 304, -1.0 }, { 312, -1.0 } };

   for (const Reference& reference : references) {
      EXPECT_EQ(frame.inphase[reference.index], reference.inphase) << "отсчёт " << reference.index;
   }
}

// Ряды и правила отображения выводятся в ответе: без них кадр невоспроизводим (Р3.17)
TEST(FrameWaveformL1OC, Test14_JsonCarriesSeriesAndRenderRules) {
   const StreamRequest request = configuration(5, 5);
   const std::string   json    = waveformFrameJson(computeWaveformFrame(request), request);

   EXPECT_TRUE(contains(json, "\"kind\": \"waveform\""));
   EXPECT_TRUE(contains(json, "\"chips\": 16"));
   EXPECT_TRUE(contains(json, "\"sampleCount\": 313"));
   EXPECT_TRUE(contains(json, "\"chipRateHz\": 1023000"));
   EXPECT_TRUE(contains(json, "\"residualFreqHz\": 0"));
   EXPECT_TRUE(contains(json, "\"zoneRule\""));
   EXPECT_TRUE(contains(json, "\"select\": 1"));
   EXPECT_TRUE(contains(json, "\"chip\""));
   EXPECT_TRUE(contains(json, "\"inphase\""));
   EXPECT_TRUE(contains(json, "\"quadrature\""));
   EXPECT_FALSE(contains(json, "nan"));
   EXPECT_FALSE(contains(json, "inf"));
}

// Изображение строится по шаблону Р3: заголовок содержательный, идентификатора kind нет,
// строки происхождения нет. Оговорка о нулевой квадратуре Q снята: она верна лишь при φ₀ = 0
TEST(FrameWaveformL1OC, Test15_SvgFollowsFrameTemplate) {
   const StreamRequest request = configuration(5, 5);
   const std::string   image   = waveformFrameSvg(computeWaveformFrame(request), request);

   EXPECT_TRUE(contains(image, "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 960 540\""));
   EXPECT_TRUE(contains(image, "<title>Осциллограмма квадратур I и Q в чиповом масштабе</title>"));
   EXPECT_TRUE(contains(image, "Чип уплотнения от начала прогона"));
   EXPECT_TRUE(contains(image, "Нормированный отсчёт u[n]"));
   EXPECT_TRUE(contains(image, "J = {5}"));
   EXPECT_TRUE(contains(image, "313 отсчётов"));
   EXPECT_TRUE(contains(image, "Квадратура I"));
   EXPECT_TRUE(contains(image, "Чипы компоненты L1OCp"));
   EXPECT_TRUE(contains(image, ">L1OCd<"));
   EXPECT_TRUE(contains(image, ">L1OCp<"));
   EXPECT_TRUE(contains(image, "</svg>"));

   EXPECT_FALSE(contains(image, "нулевая при"));
   EXPECT_FALSE(contains(image, ">Кадр waveform"));
   EXPECT_FALSE(contains(image, "signal-service-l1oc 1.0 ·"));
   EXPECT_FALSE(contains(image, "nan"));
}

// ───────────────────── кадр гистограммы мгновенных значений ─────────────────────

// Показатели прогона против независимого счёта при J = 1…24, A_j = 1, n₀ = 0.
// Отсчёт выдаётся в float32 (Д.9), поэтому пик превышает аналитическую границу η·ΣA_j
// на единицы 10⁻⁸ относительных
TEST(FrameLevelL1OC, Test16_MetricsMatchIndependentComputation) {
   const LevelFrame frame = computeLevelFrame(configuration(1, 24));

   EXPECT_NEAR(frame.eta,             0.20412414523193154, 1e-15); // 1/√24
   EXPECT_NEAR(frame.limit,           4.898979485566357,   1e-12); // η·ΣA_j = √24
   EXPECT_NEAR(frame.peak,            4.898979663848877,   1e-9);
   EXPECT_NEAR(frame.rms,             0.8918563062573858,  1e-9);
   EXPECT_NEAR(frame.crestFactor,     5.493014546712252,   1e-9);
   EXPECT_NEAR(frame.crestFactorDb,   14.796214982606300,  1e-9);
   EXPECT_NEAR(frame.axisHighPercent, 50.0,                1e-12);
   EXPECT_NEAR(frame.axisStepValue,   1.0,                 1e-12);
   EXPECT_GT(frame.peak, frame.limit); // округление float32 выводит пик за границу
}

// Заполнение корзин против независимого счёта. Квадратура I принимает 9 различных значений:
// при равных A_j символ ЦА2 — линейная форма от номера j, и сумма по J = 1…24 вырождена.
// Пять уровней приходятся ровно на границу корзины; сторону выбирает округление до float32,
// поэтому уровню −6 отвечает корзина 47, а симметричному +6
TEST(FrameLevelL1OC, Test17_BinsMatchIndependentComputation) {
   const LevelFrame frame = computeLevelFrame(configuration(1, 24));

   ASSERT_EQ(frame.counts.size(), 128U);
   ASSERT_EQ(frame.shares.size(), 128U);
   ASSERT_EQ(frame.edges.size(),  129U);
   struct Reference {
      std::size_t  index;
      std::int64_t count;
   };
   const Reference references[] = { {   0, 2999   }, {  42, 4187   }, {  47, 8208   },
      {  58, 57791  }, {  64, 115924 }, {  69, 57822  },
      {  80, 8215   }, {  85, 4054   }, { 127, 2944   } };
   std::int64_t    total = 0;
   double shareSum       = 0.0;

   for (std::size_t index = 0; index < frame.counts.size(); ++index) {
      total    += frame.counts[index];
      shareSum += frame.shares[index];
   }
   EXPECT_EQ(total, 262144);
   EXPECT_NEAR(shareSum, 100.0, 1e-9);

   std::vector<std::int64_t> expected(128, 0);

   for (const Reference& reference : references) {
      expected[reference.index] = reference.count;
   }

   for (std::size_t index = 0; index < frame.counts.size(); ++index) {
      EXPECT_EQ(frame.counts[index], expected[index]) << "корзина " << index;
   }

   // Границы корзин равномерны и симметричны относительно нуля
   EXPECT_NEAR(frame.edges.front(), -frame.limit, 1e-12);
   EXPECT_NEAR(frame.edges.back(),  frame.limit,  1e-12);
   EXPECT_NEAR(frame.edges[64],     0.0,          1e-12);
}

// Ряды и правила отображения выводятся в ответе
TEST(FrameLevelL1OC, Test18_JsonCarriesSeriesAndRenderRules) {
   const StreamRequest request = configuration(1, 3);
   const std::string   json    = levelFrameJson(computeLevelFrame(request), request);

   EXPECT_TRUE(contains(json, "\"kind\": \"level\""));
   EXPECT_TRUE(contains(json, "\"quadrature\": \"I\""));
   EXPECT_TRUE(contains(json, "\"sampleCount\": 262144"));
   EXPECT_TRUE(contains(json, "\"binCount\": 128"));
   EXPECT_TRUE(contains(json, "\"limitRule\""));
   EXPECT_TRUE(contains(json, "\"crestFactorDb\""));
   EXPECT_TRUE(contains(json, "\"rule\": \"binUniform\""));
   EXPECT_TRUE(contains(json, "\"edge\""));
   EXPECT_TRUE(contains(json, "\"count\""));
   EXPECT_TRUE(contains(json, "\"sharePercent\""));
   EXPECT_FALSE(contains(json, "nan"));
   EXPECT_FALSE(contains(json, "inf"));
}

// Изображение по шаблону Р3: показатели в поле кадра, идентификатора kind и строки
// происхождения нет
TEST(FrameLevelL1OC, Test19_SvgFollowsFrameTemplate) {
   const StreamRequest request = configuration(1, 3);
   const std::string   image   = levelFrameSvg(computeLevelFrame(request), request);

   EXPECT_TRUE(contains(image, "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 960 540\""));
   EXPECT_TRUE(contains(image, "<title>Гистограмма мгновенных значений суммарного сигнала</title>"));
   EXPECT_TRUE(contains(image, "Значение отсчёта I"));
   EXPECT_TRUE(contains(image, "Доля отсчётов, %"));
   EXPECT_TRUE(contains(image, "корзин 128"));
   EXPECT_TRUE(contains(image, "среднеквадратичное значение = "));
   EXPECT_TRUE(contains(image, "пик-фактор = "));
   EXPECT_TRUE(contains(image, "Граница шкалы η·ΣA_j"));
   EXPECT_TRUE(contains(image, "</svg>"));

   EXPECT_FALSE(contains(image, ">Кадр level"));
   EXPECT_FALSE(contains(image, "signal-service-l1oc 1.0 ·"));
   EXPECT_FALSE(contains(image, "nan"));
}

// Кадры не зависят от порядка обращений и воспроизводятся побитно на тех же параметрах
TEST(FramesL1OC, Test20_Reproducible) {
   const StreamRequest waveform = configuration(5, 5);

   EXPECT_EQ(waveformFrameSvg(computeWaveformFrame(waveform), waveform),
             waveformFrameSvg(computeWaveformFrame(waveform), waveform));
   const StreamRequest level = configuration(1, 2);

   EXPECT_EQ(levelFrameSvg(computeLevelFrame(level), level),
             levelFrameSvg(computeLevelFrame(level), level));
}

// ───────────────────── корреляционные кадры дальномерных кодов ─────────────────────

// Прореживание симметрично относительно τ = 0 (решение Р-5б): шаг — ближайшее нечётное не
// меньше ⌈4093/256⌉ = 16, центральный бин центрирован на нуле, ряд симметричен по сдвигу
TEST(FrameAcfL1OC, Test21_BinningIsSymmetric) {
   const AcfFrame frame = computeAcfFrame(configuration(1, 1));

   EXPECT_EQ(frame.decimationStep, 17);
   ASSERT_EQ(frame.codeD.bins.size(), 241U);
   ASSERT_EQ(frame.codeP.bins.size(), 241U);

   const std::size_t middle = frame.codeD.bins.size() / 2U;

   EXPECT_DOUBLE_EQ(frame.codeD.bins[middle].tauChips, 0.0);
   EXPECT_DOUBLE_EQ(frame.codeD.bins.front().tauChips, -2039.0);
   EXPECT_DOUBLE_EQ(frame.codeD.bins.back().tauChips,  2039.0);

   for (std::size_t i = 0; i < frame.codeD.bins.size(); ++i) {
      const std::size_t mirror = frame.codeD.bins.size() - 1U - i;

      EXPECT_DOUBLE_EQ(frame.codeD.bins[i].tauChips, -frame.codeD.bins[mirror].tauChips);
   }
}

// Контроль метода: битовый счёт (XOR и popcount) против прямого суммирования в {+1, −1}.
// Сверяется полное распределение боковых лепестков ПАКФ ДК_L1OCd, j = 1
TEST(FrameAcfL1OC, Test22_MatchesDirectSummation) {
   const AcfFrame frame          = computeAcfFrame(configuration(1, 1));
   const std::vector<int> symbols = codeSymbols(1, false);
   std::map<int, int> sidelobes;
   int peak = 0;

   for (int tau = 1; tau < glonass::codeLengthD; ++tau) {
      const int value = correlationDirect(symbols, symbols, tau);

      sidelobes[value] += 1;
      peak              = std::max(peak, std::abs(value));
   }
   EXPECT_EQ(frame.codeD.mainLobe,     correlationDirect(symbols, symbols, 0));
   EXPECT_EQ(frame.codeD.peakSidelobe, peak);
   ASSERT_EQ(frame.codeD.sidelobeValues.size(), sidelobes.size());
   std::size_t index = 0;

   for (const std::pair<const int, int>& item : sidelobes) {
      EXPECT_EQ(frame.codeD.sidelobeValues[index], item.first);
      EXPECT_EQ(frame.codeD.sidelobeCounts[index], item.second);
      ++index;
   }
}

// Контрольные значения независимого счёта (Python) для j = 1. Периодическое продолжение на
// ±2046 чипов даёт для ДК_L1OCd пять главных лепестков (τ = 0, ±1023, ±2046), для ДК_L1OCp —
// один: периоды кодов различаются вчетверо
TEST(FrameAcfL1OC, Test23_MatchesIndependentComputation) {
   const AcfFrame frame     = computeAcfFrame(configuration(1, 24));
   const std::size_t middle = frame.codeD.bins.size() / 2U;

   EXPECT_EQ(frame.satellite, 1);
   EXPECT_EQ(frame.codeD.lengthChips,  1023);
   EXPECT_EQ(frame.codeD.mainLobe,     1023);
   EXPECT_EQ(frame.codeD.peakSidelobe, 65);
   EXPECT_NEAR(frame.codeD.peakSidelobeDb, -23.939, 1.0e-3);
   EXPECT_EQ(frame.codeD.sidelobeValues, (std::vector<int>{ -65, -1, 63 }));
   EXPECT_EQ(frame.codeD.sidelobeCounts, (std::vector<int>{ 112, 798, 112 }));

   EXPECT_EQ(frame.codeP.lengthChips,  4092);
   EXPECT_EQ(frame.codeP.mainLobe,     4092);
   EXPECT_EQ(frame.codeP.peakSidelobe, 204);
   EXPECT_NEAR(frame.codeP.peakSidelobeDb, -26.046, 1.0e-3);
   EXPECT_EQ(frame.codeP.sidelobeValues.size(), 81U);

   EXPECT_NEAR(frame.codeD.bins.front().lowDb,     -60.198, 1.0e-3);
   EXPECT_NEAR(frame.codeD.bins.front().highDb,      0.000, 1.0e-3);
   EXPECT_NEAR(frame.codeD.bins.front().averageDb, -48.969, 1.0e-3);
   EXPECT_NEAR(frame.codeD.bins[middle].lowDb,     -60.198, 1.0e-3);
   EXPECT_NEAR(frame.codeD.bins[middle].highDb,      0.000, 1.0e-3);
   EXPECT_NEAR(frame.codeD.bins[middle].averageDb, -52.423, 1.0e-3);
   EXPECT_NEAR(frame.codeD.bins[middle + 1U].highDb, -23.939, 1.0e-3);

   EXPECT_NEAR(frame.codeP.bins.front().lowDb,     -70.000, 1.0e-3); // пол шкалы: R(τ) = 0
   EXPECT_NEAR(frame.codeP.bins.front().highDb,    -31.898, 1.0e-3);
   EXPECT_NEAR(frame.codeP.bins[middle].highDb,      0.000, 1.0e-3);
   EXPECT_NEAR(frame.codeP.bins[middle].averageDb, -34.061, 1.0e-3);

   EXPECT_DOUBLE_EQ(frame.axisLowDb,  -70.0);
   EXPECT_DOUBLE_EQ(frame.axisHighDb,   2.0);
}

// Ряды и правила отображения выводятся в ответе: без них кадр невоспроизводим (Р3.17)
TEST(FrameAcfL1OC, Test24_JsonCarriesSeriesAndRenderRules) {
   const StreamRequest request = configuration(1, 3);
   const std::string   json    = acfFrameJson(computeAcfFrame(request), request);

   EXPECT_TRUE(contains(json, "\"kind\": \"acf\""));
   EXPECT_TRUE(contains(json, "\"satellite\": 1"));
   EXPECT_TRUE(contains(json, "\"definition\": \"20·lg|R(τ)/N|\""));
   EXPECT_TRUE(contains(json, "\"source\": \"таблицы ДК блока А_L1OC\""));
   EXPECT_TRUE(contains(json, "\"spanChips\": 2046"));
   EXPECT_TRUE(contains(json, "\"icdClause\": \"2.2.1\""));
   EXPECT_TRUE(contains(json, "\"icdClause\": \"2.2.2\""));
   EXPECT_TRUE(contains(json, "\"peakSidelobe\": 65"));
   EXPECT_TRUE(contains(json, "\"sidelobeCounts\": [112, 798, 112]"));
   EXPECT_TRUE(contains(json, "\"rule\": \"binMinMaxSymmetric\""));
   EXPECT_TRUE(contains(json, "\"step\": 17"));
   EXPECT_TRUE(contains(json, "\"points\": 241"));
   EXPECT_TRUE(contains(json, "\"dbFloor\": -70"));
   EXPECT_TRUE(contains(json, "\"tauChips\""));
   EXPECT_TRUE(contains(json, "\"maxDb\""));
   EXPECT_FALSE(contains(json, "nan"));
   EXPECT_FALSE(contains(json, "inf"));
}

// Изображение по шаблону Р3: заголовок содержательный, идентификатора kind и строки
// происхождения нет, опорная линия максимума бокового лепестка подписана
TEST(FrameAcfL1OC, Test25_SvgFollowsFrameTemplate) {
   const StreamRequest request = configuration(1, 3);
   const std::string   image   = acfFrameSvg(computeAcfFrame(request), request);

   EXPECT_TRUE(contains(image, "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 960 540\""));
   EXPECT_TRUE(contains(image,
                        "<title>Периодическая автокорреляционная функция дальномерного кода</title>"));
   EXPECT_TRUE(contains(image, "НКА j = 1"));
   EXPECT_TRUE(contains(image, "коды Голда"));
   EXPECT_TRUE(contains(image, "усечённые последовательности Касами"));
   EXPECT_TRUE(contains(image, "Сдвиг τ, чипы"));
   EXPECT_TRUE(contains(image, "Уровень, дБ отн. главного лепестка"));
   EXPECT_TRUE(contains(image, "максимум бокового лепестка ДК_L1OCd: −23,939 дБ"));
   EXPECT_TRUE(contains(image, "</svg>"));

   EXPECT_FALSE(contains(image, ">Кадр acf"));
   EXPECT_FALSE(contains(image, "signal-service-l1oc 1.0 ·"));
   EXPECT_FALSE(contains(image, "nan"));
}

// Контроль метода для огибающей ансамбля: прямое суммирование по парам состава J = {1, 2, 3}
TEST(FrameCcfL1OC, Test26_MatchesDirectSummation) {
   const CcfFrame frame = computeCcfFrame(configuration(1, 3));
   std::vector<std::vector<int> > symbols;

   for (int satellite = 1; satellite <= 3; ++satellite) {
      symbols.push_back(codeSymbols(satellite, false));
   }
   std::map<int, int> levels;
   int peak   = 0;
   int lowest = glonass::codeLengthD;

   for (int tau = 0; tau < glonass::codeLengthD; ++tau) {
      int value = 0;

      for (std::size_t a = 0; a < symbols.size(); ++a) {
         for (std::size_t b = a + 1U; b < symbols.size(); ++b) {
            value = std::max(value, std::abs(correlationDirect(symbols[a], symbols[b], tau)));
         }
      }
      levels[value] += 1;
      peak           = std::max(peak, value);
      lowest         = std::min(lowest, value);
   }
   EXPECT_EQ(frame.pairCount,          3);
   EXPECT_EQ(frame.codeD.peak,         peak);
   EXPECT_EQ(frame.codeD.lowest,       lowest);
   EXPECT_EQ(frame.codeD.levelCount,   static_cast<int> (levels.size()));
   EXPECT_EQ(frame.codeD.shiftsAtPeak, levels[peak]);
}

// Контрольные значения независимого счёта (Python) для состава J = 1…24 — сценарий Д_L1OC.10
TEST(FrameCcfL1OC, Test27_MatchesIndependentComputation) {
   const CcfFrame frame     = computeCcfFrame(configuration(1, 24));
   const std::size_t middle = frame.codeD.bins.size() / 2U;

   EXPECT_EQ(frame.pairCount, 276);

   EXPECT_EQ(frame.codeD.peak,         65);
   EXPECT_NEAR(frame.codeD.peakDb, -23.939, 1.0e-3);
   EXPECT_EQ(frame.codeD.shiftsAtPeak, 1022);
   EXPECT_EQ(frame.codeD.levelCount,   2);
   EXPECT_EQ(frame.codeD.lowest,       1);
   EXPECT_NEAR(frame.codeD.lowestDb, -60.198, 1.0e-3);

   EXPECT_EQ(frame.codeP.peak,         254);
   EXPECT_NEAR(frame.codeP.peakDb, -24.142, 1.0e-3);
   EXPECT_EQ(frame.codeP.shiftsAtPeak, 1);
   EXPECT_EQ(frame.codeP.levelCount,   89);
   EXPECT_EQ(frame.codeP.lowest,       68);
   EXPECT_NEAR(frame.codeP.lowestDb, -35.589, 1.0e-3);

   EXPECT_NEAR(frame.codeD.bins[middle].lowDb,     -60.198, 1.0e-3);
   EXPECT_NEAR(frame.codeD.bins[middle].highDb,    -23.939, 1.0e-3);
   EXPECT_NEAR(frame.codeD.bins[middle].averageDb, -26.072, 1.0e-3);
   EXPECT_NEAR(frame.codeP.bins[middle].lowDb,     -35.589, 1.0e-3);
   EXPECT_NEAR(frame.codeP.bins[middle].highDb,    -34.397, 1.0e-3);
   EXPECT_NEAR(frame.codeP.bins[middle].averageDb, -35.056, 1.0e-3);

   EXPECT_DOUBLE_EQ(frame.axisLowDb,  -70.0);
   EXPECT_DOUBLE_EQ(frame.axisHighDb, -20.0);
}

// Состав из одного НКА: пар нет, кадр не определён — отказ разряда «нереализуемо» (422)
TEST(FrameCcfL1OC, Test28_RejectsSingleSatellite) {
   try {
      computeCcfFrame(configuration(5, 5));
      FAIL() << "ожидался отказ по составу";
   } catch (const glonass_params::ParamError& error) {
      EXPECT_EQ(error.kind(),  glonass_params::RejectKind::unrealizable);
      EXPECT_EQ(error.field(), "j");
      EXPECT_TRUE(contains(error.what(), "в составе один НКА"));
   }
}

// Ряды и правила отображения выводятся в ответе (Р3.17)
TEST(FrameCcfL1OC, Test29_JsonCarriesSeriesAndRenderRules) {
   const StreamRequest request = configuration(1, 3);
   const std::string   json    = ccfFrameJson(computeCcfFrame(request), request);

   EXPECT_TRUE(contains(json, "\"kind\": \"ccf\""));
   EXPECT_TRUE(contains(json, "\"satelliteCount\": 3"));
   EXPECT_TRUE(contains(json, "\"pairCount\": 3"));
   EXPECT_TRUE(contains(json, "\"definition\": \"env(τ) = max |R_ab(τ)| по парам a < b состава J\""));
   EXPECT_TRUE(contains(json, "\"source\": \"таблицы ДК блока А_L1OC\""));
   EXPECT_TRUE(contains(json, "\"shiftsAtPeak\""));
   EXPECT_TRUE(contains(json, "\"levelCount\""));
   EXPECT_TRUE(contains(json, "\"rule\": \"binMinMaxSymmetric\""));
   EXPECT_TRUE(contains(json, "\"points\": 241"));
   EXPECT_TRUE(contains(json, "\"minDb\""));
   EXPECT_TRUE(contains(json, "\"avgDb\""));
   EXPECT_FALSE(contains(json, "nan"));
   EXPECT_FALSE(contains(json, "inf"));
}

// Изображение по шаблону Р3; состав при |J| ≤ 4 выводится перечислением
TEST(FrameCcfL1OC, Test30_SvgFollowsFrameTemplate) {
   const StreamRequest request = configuration(1, 3);
   const std::string   image   = ccfFrameSvg(computeCcfFrame(request), request);

   EXPECT_TRUE(contains(image, "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 960 540\""));
   EXPECT_TRUE(contains(image,
                        "<title>Огибающая периодической взаимнокорреляционной функции ансамбля</title>"));
   EXPECT_TRUE(contains(image, "J = {1, 2, 3}"));
   EXPECT_TRUE(contains(image, "пар 3"));
   EXPECT_TRUE(contains(image, "Сдвиг τ, чипы"));
   EXPECT_TRUE(contains(image, "Уровень, дБ отн. длины кода"));
   EXPECT_TRUE(contains(image, "сдвигов на максимуме"));
   EXPECT_TRUE(contains(image, "</svg>"));

   EXPECT_FALSE(contains(image, ">Кадр ccf"));
   EXPECT_FALSE(contains(image, "signal-service-l1oc 1.0 ·"));
   EXPECT_FALSE(contains(image, "nan"));
}

// ───────────────────── кадр строки навигационного сообщения ─────────────────────

// Строка и её показатели против независимого счёта (Python, frame_navline_l1oc.py): нормальная
// строка при нулевой ЦИ, состояние регистра СК при запуске нулевое (§ 0.1 поз.41).
// Символы 12…23 — контрольное значение ИКД п. 4.2.2.1
TEST(FrameNavLineL1OC, Test31_LineMatchesIndependentComputation) {
   const NavLineFrame frame = computeNavLineFrame(configuration(1, 3));

   EXPECT_EQ(frame.lineType,     glonass::LineTypeL1OC::normal);
   EXPECT_EQ(frame.infoBits,     222);
   EXPECT_EQ(frame.lineBitCount, 250);
   EXPECT_EQ(frame.lineLength,   500);
   EXPECT_EQ(static_cast<int> (frame.lineSymbols.size()), 500);

   EXPECT_EQ(symbolText(frame.lineSymbols, 0, 24), "001101000101000111011010");
   EXPECT_EQ(symbolText(frame.lineSymbols, 12, 12), "000111011010");
   EXPECT_EQ(symbolText(frame.lineSymbols, 476, 24), "001011101100010001010101");

   EXPECT_EQ(frame.onesCount,   32);
   EXPECT_EQ(frame.transitions, 39);

   const glonass::ConvStateL1OC expectedState = { 0, 0, 1, 0, 1, 1 };

   EXPECT_EQ(frame.convStateOut, expectedState);

   EXPECT_DOUBLE_EQ(frame.symbolDurationMs, 4.0);
   EXPECT_DOUBLE_EQ(frame.lineDurationS,    2.0);
}

// Зоны полей на оси символов СК: бит t информационного блока даёт символы 2t, 2t+1.
// Единицы сосредоточены в СМВ и ЦК; в зоне ЦИ их пять — след памяти кодера после СМВ
TEST(FrameNavLineL1OC, Test32_FieldsMatchLineStructure) {
   const NavLineFrame frame = computeNavLineFrame(configuration(1, 3));

   ASSERT_EQ(frame.fields.size(), 3U);

   const double from[3] = { 0.0, 24.0, 468.0 };
   const double to[3]   = { 24.0, 468.0, 500.0 };
   const int bits[3]    = { 12, 222, 16 };
   const char* name[3]  = { "СМВ", "ЦИ", "ЦК" };
   const int ones[3]    = { 11, 5, 16 };

   for (std::size_t index = 0; index < frame.fields.size(); ++index) {
      EXPECT_DOUBLE_EQ(frame.fields[index].from, from[index]);
      EXPECT_DOUBLE_EQ(frame.fields[index].to,   to[index]);
      EXPECT_EQ(frame.fields[index].bits, bits[index]);
      EXPECT_STREQ(frame.fields[index].name, name[index]);
      EXPECT_EQ(onesWithin(frame.lineSymbols, frame.fields[index].from, frame.fields[index].to),
                ones[index]);
   }
}

// Координаты строки от привязки: ℓ, w[n₀] и фаза символа против независимого счёта.
// Слой содержания сервиса выдаёт нулевую ЦИ на любой ℓ, поэтому кадр при разных n₀
// отличается только положением отметки
TEST(FrameNavLineL1OC, Test33_CoordinatesMatchIndependentComputation) {
   struct Case {
      std::int64_t sampleRate;
      std::int64_t startSample;
      std::int64_t lineIndex;
      int          convSymbolIndex;
      double       symbolPhase;
   };
   const Case cases[] = { { 20000000, 0, 0, 0, 0.0 },
                          { 20000000, 12345, 0, 0, 0.154312 },
                          { 20000000, 20000000, 0, 250, 0.0 },
                          { 20000000, 45000000, 1, 62, 0.5 },
                          { 20000000, 123456789, 3, 43, 0.209863 },
                          { 4092000, 1234567, 0, 75, 0.425648 },
                          { 5000000, 999999999, 99, 499, 0.999950 } };
   const NavLineFrame reference = computeNavLineFrame(configuration(1, 3));

   for (const Case& item : cases) {
      const NavLineFrame frame =
         computeNavLineFrame(configurationAt(item.sampleRate, item.startSample));

      EXPECT_EQ(frame.lineIndex,       item.lineIndex);
      EXPECT_EQ(frame.convSymbolIndex, item.convSymbolIndex);
      EXPECT_NEAR(frame.symbolPhase, item.symbolPhase, 1.0e-6);
      EXPECT_EQ(frame.lineSymbols, reference.lineSymbols);
   }
}

// n₀·R_с вне разрядности счётчика символов: отказ того же разряда, что в режиме А (400)
TEST(FrameNavLineL1OC, Test34_RejectsStartSampleBeyondSymbolCounter) {
   const std::int64_t limit = std::numeric_limits<std::int64_t>::max() / glonass::symbolRateL1OC;

   try {
      computeNavLineFrame(configurationAt(20000000, limit + 1));
      FAIL() << "ожидался отказ по разрядности счётчика символов";
   } catch (const glonass_params::ParamError& error) {
      EXPECT_EQ(error.kind(),  glonass_params::RejectKind::badValue);
      EXPECT_EQ(error.field(), "n0");
      EXPECT_TRUE(contains(error.what(), "вне разрядности счётчика символов"));
   }
}

// Ряды и правила отображения выводятся в ответе: без них кадр невоспроизводим (Р3.17)
TEST(FrameNavLineL1OC, Test35_JsonCarriesSeriesAndRenderRules) {
   const StreamRequest request = configurationAt(20000000, 45000000);
   const std::string   json    = navLineFrameJson(computeNavLineFrame(request), request);

   EXPECT_TRUE(contains(json, "\"kind\": \"navline\""));
   EXPECT_TRUE(contains(json, "\"lineIndex\": 1"));
   EXPECT_TRUE(contains(json, "\"lineType\": \"normal\""));
   EXPECT_TRUE(contains(json, "\"smvBits\": 12"));
   EXPECT_TRUE(contains(json, "\"infoBits\": 222"));
   EXPECT_TRUE(contains(json, "\"lineBits\": 250"));
   EXPECT_TRUE(contains(json, "\"lineLength\": 500"));
   EXPECT_TRUE(contains(json, "\"symbolRateHz\": 250"));
   EXPECT_TRUE(contains(json, "\"convSymbolIndex\": 62"));
   EXPECT_TRUE(contains(json, "\"convStateOut\": [0, 0, 1, 0, 1, 1]"));
   EXPECT_TRUE(contains(json, "\"onesCount\": 32"));
   EXPECT_TRUE(contains(json, "\"transitions\": 39"));
   EXPECT_TRUE(contains(json, "\"icdClause\": \"4.2.2.1, 4.4\""));
   EXPECT_TRUE(contains(json, "\"levelRule\": \"уровень 1 − 2·b_line\""));
   EXPECT_TRUE(contains(json, "\"marker\": 62"));
   EXPECT_TRUE(contains(json, "\"name\": \"СМВ\""));
   EXPECT_TRUE(contains(json, "\"name\": \"ЦИ\""));
   EXPECT_TRUE(contains(json, "\"name\": \"ЦК\""));
   EXPECT_TRUE(contains(json, "\"symbol\": [0, 0, 1, 1, 0, 1"));
   EXPECT_FALSE(contains(json, "nan"));
   EXPECT_FALSE(contains(json, ": inf")); // «inf» как значение: ключ infoBits несёт ту же подстроку
}

// Изображение строится по шаблону Р3: заголовок содержательный, идентификатора kind нет,
// строки происхождения нет; состав при |J| ≤ 4 выводится перечислением
TEST(FrameNavLineL1OC, Test36_SvgFollowsFrameTemplate) {
   const StreamRequest request = configurationAt(20000000, 45000000);
   const std::string   image   = navLineFrameSvg(computeNavLineFrame(request), request);

   EXPECT_TRUE(contains(image, "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 960 540\""));
   EXPECT_TRUE(contains(image,
                        "<title>Строка навигационного сообщения: символы свёрточного кода</title>"));
   EXPECT_TRUE(contains(image, "J = {1, 2, 3}"));
   EXPECT_TRUE(contains(image, "Строка ℓ = 1"));
   EXPECT_TRUE(contains(image, "нормальная строка"));
   EXPECT_TRUE(contains(image, "w[n₀] = 62"));
   EXPECT_TRUE(contains(image, "Символ свёрточного кода в строке"));
   EXPECT_TRUE(contains(image, "Символ b_line в алфавите {−1, +1}"));
   EXPECT_TRUE(contains(image, ">СМВ<"));
   EXPECT_TRUE(contains(image, ">ЦК<"));
   EXPECT_TRUE(contains(image, "</svg>"));

   EXPECT_FALSE(contains(image, ">Кадр navline"));
   EXPECT_FALSE(contains(image, "signal-service-l1oc 1.0 ·"));
   EXPECT_FALSE(contains(image, "nan"));
}

// Кадр не зависит от порядка обращений и воспроизводится побитно на тех же параметрах
TEST(FrameNavLineL1OC, Test37_Reproducible) {
   const StreamRequest request = configurationAt(20000000, 123456789);

   EXPECT_EQ(navLineFrameSvg(computeNavLineFrame(request), request),
             navLineFrameSvg(computeNavLineFrame(request), request));
   EXPECT_EQ(navLineFrameJson(computeNavLineFrame(request), request),
             navLineFrameJson(computeNavLineFrame(request), request));
}
