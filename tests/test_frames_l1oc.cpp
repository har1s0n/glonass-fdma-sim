#include "fft_radix2.h"
#include "frame_psd.h"
#include "svg_canvas.h"

#include "glonass/types.h"

#include <gtest/gtest.h>
#include <cmath>
#include <complex>
#include <cstddef>
#include <string>
#include <vector>

// Кадры микросервиса L1OC — базовый слой и кадр спектральной плотности мощности

namespace {
using glonass_service::computePsdFrame;
using glonass_service::dftDirect;
using glonass_service::fftRadix2;
using glonass_service::numberRu;
using glonass_service::PsdFrame;
using glonass_service::psdFrameJson;
using glonass_service::psdFrameSvg;
using glonass_service::StreamRequest;

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

// Сверка с прямым дискретным преобразованием: тот же приём, что в fft_check.py
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

// Параметры оценки и прореживания — по решению У5: 8192 × 32, предел 2000 точек,
// шаг прореживания округляется вверх (8192/2000 → 5 → 1639 точек)
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
