#include "fft_radix2.h"
#include "frame_level.h"
#include "frame_psd.h"
#include "frame_waveform.h"
#include "svg_canvas.h"

#include "glonass/types.h"

#include <gtest/gtest.h>
#include <cmath>
#include <complex>
#include <cstddef>
#include <string>
#include <vector>

// Кадры микросервиса L1OC — базовый слой, кадры спектральной плотности мощности,
// осциллограммы квадратур и гистограммы мгновенных значений

namespace {
using glonass_service::computeLevelFrame;
using glonass_service::computePsdFrame;
using glonass_service::computeWaveformFrame;
using glonass_service::dftDirect;
using glonass_service::fftRadix2;
using glonass_service::LevelFrame;
using glonass_service::levelFrameJson;
using glonass_service::levelFrameSvg;
using glonass_service::numberRu;
using glonass_service::PsdFrame;
using glonass_service::psdFrameJson;
using glonass_service::psdFrameSvg;
using glonass_service::StreamRequest;
using glonass_service::WaveformFrame;
using glonass_service::waveformFrameJson;
using glonass_service::waveformFrameSvg;

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
