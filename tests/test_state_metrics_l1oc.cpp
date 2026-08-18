#include "state_metrics.h"

#include "glonass/types.h"

#include <gtest/gtest.h>
#include <cstdint>
#include <string>
#include <vector>

// Показатели режима А микросервиса L1OC (GET /v1/state)

namespace {
using glonass_params::ParamError;
using glonass_params::RejectKind;
using glonass_service::computeStateMetrics;
using glonass_service::parseStateRequest;
using glonass_service::StateMetrics;
using glonass_service::StateRequest;

constexpr std::int64_t carrier = glonass::carrierFreqL1OC;

std::vector<int> range(int first, int last) {
   std::vector<int> values;

   for (int value = first; value <= last; ++value) {
      values.push_back(value);
   }
   return values;
}

// Конфигурация запроса: J = {first…last}, A_j = 1, φ_{0,j} = 0
StateRequest configuration(std::int64_t sampleRate, std::int64_t referenceFreq,
                           std::int64_t startSample, int first, int last, double time) {
   StateRequest request;

   request.sampleRate    = sampleRate;
   request.referenceFreq = referenceFreq;
   request.startSample   = startSample;
   request.satellites    = range(first, last);
   request.amplitudes    = std::vector<double> (request.satellites.size(), 1.0);
   request.initialPhases = std::vector<double> (request.satellites.size(), 0.0);
   request.time          = time;
   return request;
}

struct Rejection {
   bool        thrown = false;
   RejectKind  kind   = RejectKind::badValue;
   std::string field;
};

template<class Action>
Rejection rejectionOf(Action action) {
   try {
      action();
   } catch (const ParamError& error) {
      return Rejection{ true, error.kind(), error.field() };
   }
   return Rejection{};
}
} // namespace

// ───────────────── контрольная таблица К1…К12 ─────────────────

// К1 — опорная: 24 НКА, полсекунды внутрь строки 6
TEST(StateMetrics, Test1_ReferenceConfiguration) {
   const StateMetrics metrics = computeStateMetrics(configuration(20000000, carrier, 0, 1, 24, 12.5));

   EXPECT_EQ(metrics.sampleIndex, 250000000);
   EXPECT_DOUBLE_EQ(metrics.time, 12.5);
   EXPECT_EQ(metrics.satelliteCount, 24u);
   EXPECT_DOUBLE_EQ(metrics.normalizationFactor, 0.20412414523193154);
   EXPECT_EQ(metrics.modelBandwidthHz, 2046000);
   EXPECT_EQ(metrics.residualFreqHz,   0);
   EXPECT_TRUE(metrics.representable);
   EXPECT_EQ(metrics.lineIndex,       6);
   EXPECT_EQ(metrics.lineType,        glonass::LineTypeL1OC::normal);
   EXPECT_EQ(metrics.convSymbolIndex, 125);
   EXPECT_EQ(metrics.lineLength,      500);
}

// К2 — одиночный источник: η = 1 точно (Д_L1OC.1 при |J| = 1)
TEST(StateMetrics, Test2_SingleSourceNormalizationIsExactlyOne) {
   const StateMetrics metrics = computeStateMetrics(configuration(20000000, carrier, 0, 1, 1, 0.0));

   EXPECT_EQ(metrics.sampleIndex,         0);
   EXPECT_EQ(metrics.satelliteCount,      1u);
   EXPECT_EQ(metrics.normalizationFactor, 1.0); // точное значение, не приближение
   EXPECT_EQ(metrics.lineIndex,           0);
   EXPECT_EQ(metrics.convSymbolIndex,     0);
}

// К3 — неравные амплитуды: η = 1/√5 (Д_L1OC.1); √|J| здесь неприменимо
TEST(StateMetrics, Test3_UnequalAmplitudes) {
   StateRequest request = configuration(20000000, carrier, 0, 1, 2, 0.0);

   request.amplitudes = { 1.0, 2.0 };

   const StateMetrics metrics = computeStateMetrics(request);

   EXPECT_EQ(metrics.satelliteCount, 2u);
   EXPECT_DOUBLE_EQ(metrics.normalizationFactor, 0.44721359549995793);
}

// К4, К5 — граница строки: w: 499 → 0 при переходе через 2 с
TEST(StateMetrics, Test4_LineBoundary) {
   const StateMetrics atBoundary = computeStateMetrics(configuration(20000000, carrier, 0, 1, 24, 2.0));

   EXPECT_EQ(atBoundary.sampleIndex,     40000000);
   EXPECT_EQ(atBoundary.lineIndex,       1);
   EXPECT_EQ(atBoundary.convSymbolIndex, 0);

   const StateMetrics beforeBoundary =
      computeStateMetrics(configuration(20000000, carrier, 0, 1, 24, 1.996));

   EXPECT_EQ(beforeBoundary.sampleIndex,     39920000);
   EXPECT_EQ(beforeBoundary.lineIndex,       0);
   EXPECT_EQ(beforeBoundary.convSymbolIndex, 499);
}

// К6 — привязка n₀ входит в индекс отсчёта наравне с модельным временем (§ 0.1 поз.25)
TEST(StateMetrics, Test5_StartSampleShiftsIndex) {
   const StateMetrics metrics =
      computeStateMetrics(configuration(20000000, carrier, 20000000, 1, 1, 1.0));

   EXPECT_EQ(metrics.sampleIndex,     40000000);
   EXPECT_EQ(metrics.lineIndex,       1);
   EXPECT_EQ(metrics.convSymbolIndex, 0);
}

// К7, К8 — расстройка учитывается вместе с полосой модели (В.2)
TEST(StateMetrics, Test6_RepresentabilityReportedNotRejected) {
   const std::int64_t shifted = carrier - 1000000;
   const StateMetrics wide    = computeStateMetrics(configuration(20000000, shifted, 0, 1, 1, 0.0));

   EXPECT_EQ(wide.residualFreqHz, 1000000);
   EXPECT_TRUE(wide.representable);

   const StateMetrics narrow = computeStateMetrics(configuration(5000000, shifted, 0, 1, 1, 0.0));

   EXPECT_EQ(narrow.residualFreqHz, 1000000);
   EXPECT_FALSE(narrow.representable);

   // Прочие поля сохраняют смысл: показатели выдаются и при невыполненном условии
   EXPECT_EQ(narrow.modelBandwidthHz, 2046000);
   EXPECT_EQ(narrow.lineLength,       500);
}

// К9, К10 — граница В.2 нестрогая: равенство принимается
TEST(StateMetrics, Test7_BandBoundaryIsInclusive) {
   EXPECT_TRUE(computeStateMetrics(configuration(4092000, carrier, 0, 1, 1, 0.0)).representable);
   EXPECT_FALSE(computeStateMetrics(configuration(4091999, carrier, 0, 1, 1, 0.0)).representable);
}

// К11 — полное множество системных номеров
TEST(StateMetrics, Test8_FullSatelliteSet) {
   const StateMetrics metrics = computeStateMetrics(configuration(20000000, carrier, 0, 1, 63, 0.0));

   EXPECT_EQ(metrics.satelliteCount, 63u);
   EXPECT_DOUBLE_EQ(metrics.normalizationFactor, 0.12598815766974239);
}

// К12 — округление n = n₀ + round(t·Fs): половина ОТ нуля (§ 0.1 поз.20), не к чётному
TEST(StateMetrics, Test9_HalfRoundsAwayFromZero) {
   const StateMetrics metrics =
      computeStateMetrics(configuration(20000000, carrier, 0, 1, 1, 1.25e-07));

   EXPECT_EQ(metrics.sampleIndex, 3); // t·Fs = 2,5 ровно; округление к чётному дало бы 2
}

// ───────────────── отказы расчёта ─────────────────

// Формулы Б_L1OC.4(4) выведены для n ≥ 0
TEST(StateMetrics, Test10_NegativeSampleIndexRejected) {
   const Rejection rejection = rejectionOf([] {
      computeStateMetrics(configuration(20000000, carrier, 0, 1, 1, -0.0045));
   });

   EXPECT_TRUE(rejection.thrown);
   EXPECT_EQ(rejection.kind,  RejectKind::badValue);
   EXPECT_EQ(rejection.field, "t");
}

TEST(StateMetrics, Test11_TimeOutOfRangeRejected) {
   for (const double time : { 1.0e18, -1.0e18 }) {
      const Rejection rejection = rejectionOf([time] {
         computeStateMetrics(configuration(20000000, carrier, 0, 1, 1, time));
      });

      EXPECT_TRUE(rejection.thrown) << time;
      EXPECT_EQ(rejection.kind, RejectKind::badValue) << time;
   }
}

// ───────────────── сериализация ответа ─────────────────

// Состав и порядок полей § 5.1 контракта; вещественные — представлением round-trip double
TEST(StateMetricsJson, Test1_ReferenceBody) {
   const StateMetrics metrics = computeStateMetrics(configuration(20000000, carrier, 0, 1, 24, 12.5));

   EXPECT_EQ(glonass_service::stateMetricsJson(metrics),
             "{\"n\": 250000000, \"t\": 12.5, \"band\": \"L1OC\", \"satelliteCount\": 24, "
             "\"normalizationFactor\": 0.20412414523193154, \"modelBandwidthHz\": 2046000, "
             "\"residualFreqHz\": 0, \"representable\": true, "
             "\"message\": {\"lineIndex\": 6, \"lineType\": \"normal\", "
             "\"convSymbolIndex\": 125, \"lineLength\": 500}}");
}

TEST(StateMetricsJson, Test2_LineTypeNames) {
   EXPECT_STREQ(glonass_service::lineTypeName(glonass::LineTypeL1OC::normal),     "normal");
   EXPECT_STREQ(glonass_service::lineTypeName(glonass::LineTypeL1OC::anomalous1), "anomalous1");
   EXPECT_STREQ(glonass_service::lineTypeName(glonass::LineTypeL1OC::anomalous2), "anomalous2");
}

// ───────────────── разбор строки запроса ─────────────────

// Умолчания § 4 контракта применяются при пустой строке запроса
TEST(StateRequestParsing, Test1_DefaultsMatchLaunchModule) {
   const httplib::Request request;
   const StateRequest     parsed = parseStateRequest(request);

   EXPECT_EQ(parsed.sampleRate,    20000000);
   EXPECT_EQ(parsed.referenceFreq, carrier);
   EXPECT_EQ(parsed.startSample,   0);
   EXPECT_EQ(parsed.satellites,    range(1, 24));
   EXPECT_EQ(parsed.amplitudes,    std::vector<double> (24, 1.0));
   EXPECT_EQ(parsed.initialPhases, std::vector<double> (24, 0.0));
   EXPECT_DOUBLE_EQ(parsed.time, 0.0);
}

TEST(StateRequestParsing, Test2_ParametersRead) {
   httplib::Request request;

   request.params.emplace("fs",   "10000000");
   request.params.emplace("f0",   std::to_string(carrier - 1000000));
   request.params.emplace("n0",   "20000000");
   request.params.emplace("j",    "3,1");
   request.params.emplace("amp",  "1,2");
   request.params.emplace("phi0", "0.5");
   request.params.emplace("t",    "12.5");

   const StateRequest parsed = parseStateRequest(request);

   EXPECT_EQ(parsed.sampleRate,    10000000);
   EXPECT_EQ(parsed.referenceFreq, carrier - 1000000);
   EXPECT_EQ(parsed.startSample,   20000000);
   EXPECT_EQ(parsed.satellites,    std::vector<int> ({ 3, 1 }));
   EXPECT_EQ(parsed.amplitudes,    std::vector<double> ({ 1.0, 2.0 }));
   EXPECT_EQ(parsed.initialPhases, std::vector<double> ({ 0.5, 0.5 }));
   EXPECT_DOUBLE_EQ(parsed.time, 12.5);
}

// Параметры формы выдачи на показатели не влияют и в режиме А не разбираются
TEST(StateRequestParsing, Test3_OutputShapeParametersIgnored) {
   httplib::Request request;

   request.params.emplace("n",            "не число");
   request.params.emplace("seconds",      "не число");
   request.params.emplace("format",       "cs16");
   request.params.emplace("blockSamples", "65536");

   EXPECT_NO_THROW(parseStateRequest(request));
}

// Условие представимости при разборе НЕ проверяется: точка прогона не выполняет
TEST(StateRequestParsing, Test4_RepresentabilityNotRejectedOnParse) {
   httplib::Request request;

   request.params.emplace("fs", "4091999");
   EXPECT_NO_THROW(parseStateRequest(request));
   EXPECT_FALSE(computeStateMetrics(parseStateRequest(request)).representable);
}

// Предусловие Б_L1OC.8 проверяется и в режиме А: координаты сообщения выводятся из Fs
TEST(StateRequestParsing, Test5_SymbolRatePreconditionRejected) {
   httplib::Request request;

   request.params.emplace("fs", "249");

   const Rejection rejection = rejectionOf([&request] {
      parseStateRequest(request);
   });

   EXPECT_TRUE(rejection.thrown);
   EXPECT_EQ(rejection.kind,  RejectKind::unrealizable);
   EXPECT_EQ(rejection.field, "fs");
}

TEST(StateRequestParsing, Test6_FieldNamesMatchQueryKeys) {
   const struct {
      const char* key;
      const char* value;
      RejectKind  kind;
   } cases[] = {
      { "fs",   "0",      RejectKind::badValue              },
      { "n0",   "-1",     RejectKind::badValue              },
      { "j",    "0",      RejectKind::badValue              },
      { "amp",  "-1",     RejectKind::badValue              },
      { "amp",  "0",      RejectKind::unrealizable          },
      { "phi0", "1,2,3",  RejectKind::badValue              },
      { "t",    "abc",    RejectKind::badValue              },
   };

   for (const auto& item : cases) {
      httplib::Request request;

      request.params.emplace("j", "1,2"); // |J| = 2 — для проверки длины списков
      request.params.erase(item.key);
      request.params.emplace(item.key, item.value);

      const Rejection rejection = rejectionOf([&request] {
         parseStateRequest(request);
      });

      EXPECT_TRUE(rejection.thrown) << item.key << '=' << item.value;
      EXPECT_EQ(rejection.kind,  item.kind) << item.key << '=' << item.value;
      EXPECT_EQ(rejection.field, item.key) << item.key << '=' << item.value;
   }
}
