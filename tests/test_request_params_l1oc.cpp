#include "request_params_l1oc.h"

#include "glonass/types.h"

#include <gtest/gtest.h>
#include <string>
#include <vector>

// Общий разбор параметров прогона тракта L1OC (apps/common/request_params_l1oc.h)

namespace {
using glonass_params::ParamError;
using glonass_params::RejectKind;

struct Rejection {
   bool        thrown = false;
   RejectKind  kind   = RejectKind::badValue;
   std::string field;
   std::string message;
};

template<class Action>
Rejection rejectionOf(Action action) {
   try {
      action();
   } catch (const ParamError& error) {
      return Rejection{ true, error.kind(), error.field(), error.what() };
   }
   return Rejection{};
}

std::vector<int> range(int first, int last) {
   std::vector<int> values;

   for (int value = first; value <= last; ++value) {
      values.push_back(value);
   }
   return values;
}
} // namespace

// ───────────────────────── j: состав активных НКА ─────────────────────────

// Диапазон a:b раскрывается включительно; умолчание § 4 контракта — «1:24»
TEST(RequestParamsSatellites, Test1_RangeExpandsInclusive) {
   EXPECT_EQ(glonass_params::parseSatellites("1:24", "j"),                            range(1, 24));
   EXPECT_EQ(glonass_params::parseSatellites(glonass_params::defaultSatellites, "j"), range(1, 24));
   EXPECT_EQ(glonass_params::parseSatellites("7:7", "j"),                             std::vector<int> ({ 7 }));
   EXPECT_EQ(glonass_params::parseSatellites("1:63", "j").size(),                     63u);
}

// Список сохраняет порядок следования: сортировка выполняется источником сигнала (Д_L1OC.8)
TEST(RequestParamsSatellites, Test2_ListKeepsOrder) {
   EXPECT_EQ(glonass_params::parseSatellites("3,1,2", "j"), std::vector<int> ({ 3, 1, 2 }));
   EXPECT_EQ(glonass_params::parseSatellites("5", "j"),     std::vector<int> ({ 5 }));
}

// j = 0 резервный, j ≥ 64 вне диапазона (§ 0.1 поз.28)
TEST(RequestParamsSatellites, Test3_SystemNumberOutOfRangeRejected) {
   for (const char* text : { "0", "0:3", "64", "1:64", "-1" }) {
      const Rejection rejection = rejectionOf([text] {
         glonass_params::parseSatellites(text, "j");
      });

      EXPECT_TRUE(rejection.thrown) << text;
      EXPECT_EQ(rejection.kind,  RejectKind::badValue) << text;
      EXPECT_EQ(rejection.field, "j") << text;
   }
   EXPECT_EQ(glonass_params::parseSatellites("63", "j"), std::vector<int> ({ 63 }));
}

// J — множество (Д_L1OC.2): повтор системного номера отклоняется
TEST(RequestParamsSatellites, Test4_DuplicateRejected) {
   const Rejection rejection = rejectionOf([] {
      glonass_params::parseSatellites("1,2,1", "j");
   });

   EXPECT_TRUE(rejection.thrown);
   EXPECT_EQ(rejection.kind, RejectKind::badValue);
   EXPECT_NE(rejection.message.find("повторяющийся"), std::string::npos);
}

TEST(RequestParamsSatellites, Test5_MalformedRejected) {
   for (const char* text : { "", "abc", "5abc", "1:2:3", "5:3", "1,,2" }) {
      const Rejection rejection = rejectionOf([text] {
         glonass_params::parseSatellites(text, "j");
      });

      EXPECT_TRUE(rejection.thrown) << text;
      EXPECT_EQ(rejection.kind, RejectKind::badValue) << text;
   }
}

// Ключ передаётся вызывающим: тексты сообщений модуля запуска сохраняются
TEST(RequestParamsSatellites, Test6_KeyAppearsInMessageAndField) {
   const Rejection cli = rejectionOf([] {
      glonass_params::parseSatellites("0", "--j");
   });

   EXPECT_EQ(cli.field,   "--j");
   EXPECT_EQ(cli.message, "--j: системный номер вне {1,…,63} (j = 0 резервный): 0");
}

// ───────────────────── amp / phi0: значения по источникам ─────────────────────

// Скаляр применяется ко всем НКА, список — по |J|
TEST(RequestParamsPerSatellite, Test1_ScalarAndList) {
   EXPECT_EQ(glonass_params::parsePerSatellite("1", 3, "amp"),    std::vector<double> ({ 1.0, 1.0, 1.0 }));
   EXPECT_EQ(glonass_params::parsePerSatellite("1,2", 2, "amp"),  std::vector<double> ({ 1.0, 2.0 }));
   EXPECT_EQ(glonass_params::parsePerSatellite("0.5", 1, "phi0"), std::vector<double> ({ 0.5 }));
}

TEST(RequestParamsPerSatellite, Test2_LengthMismatchRejected) {
   const Rejection rejection = rejectionOf([] {
      glonass_params::parsePerSatellite("1,2,3", 2, "amp");
   });

   EXPECT_TRUE(rejection.thrown);
   EXPECT_EQ(rejection.kind,    RejectKind::badValue);
   EXPECT_EQ(rejection.field,   "amp");
   EXPECT_EQ(rejection.message, "amp: число значений != |J|");
}

TEST(RequestParamsPerSatellite, Test3_MalformedRejected) {
   for (const char* text : { "", "abc", "1.5x" }) {
      const Rejection rejection = rejectionOf([text] {
         glonass_params::parsePerSatellite(text, 1, "amp");
      });

      EXPECT_TRUE(rejection.thrown) << text;
      EXPECT_EQ(rejection.kind, RejectKind::badValue) << text;
   }
}

// A_j ≥ 0 — диапазон значения (§ 0.1 поз.24), 400; Σ A_j² > 0 — предусловие Д_L1OC.1, 422
TEST(RequestParamsAmplitudes, Test1_NegativeIsBadValue) {
   const Rejection rejection = rejectionOf([] {
      glonass_params::requireAmplitudes({ 1.0, -1.0 }, "amp");
   });

   EXPECT_TRUE(rejection.thrown);
   EXPECT_EQ(rejection.kind,  RejectKind::badValue);
   EXPECT_EQ(rejection.field, "amp");
}

TEST(RequestParamsAmplitudes, Test2_ZeroSumIsUnrealizable) {
   const Rejection rejection = rejectionOf([] {
      glonass_params::requireAmplitudes({ 0.0, 0.0 }, "amp");
   });

   EXPECT_TRUE(rejection.thrown);
   EXPECT_EQ(rejection.kind, RejectKind::unrealizable);
   EXPECT_NE(rejection.message.find("Д_L1OC.1"), std::string::npos);
}

TEST(RequestParamsAmplitudes, Test3_AdmissibleAccepted) {
   EXPECT_NO_THROW(glonass_params::requireAmplitudes({ 1.0, 2.0 }, "amp"));
   EXPECT_NO_THROW(glonass_params::requireAmplitudes({ 0.0, 1.0 }, "amp")); // нулевая амплитуда допустима
}

// ─────────────────── условия В.2 и Б_L1OC.8, привязка n₀ ───────────────────

// Δf_j = f_L1OC − f₀; при f₀ = f_L1OC ⇒ Δf_j = 0 (§ 0.1 поз.26)
TEST(RequestParamsRepresentable, Test1_ResidualFreq) {
   EXPECT_EQ(glonass_params::residualFreq(glonass::carrierFreqL1OC),           0);
   EXPECT_EQ(glonass_params::residualFreq(glonass::carrierFreqL1OC - 1000000), 1000000);
   EXPECT_EQ(glonass_params::residualFreq(glonass::carrierFreqL1OC + 1000000), -1000000);
}

// Границы В.2: равенство принимается (К9), на 1 Гц ниже — отказ (К10)
TEST(RequestParamsRepresentable, Test2_BandBoundaryIsInclusive) {
   EXPECT_TRUE(glonass_params::isRepresentable(4092000, glonass::carrierFreqL1OC));  // К9
   EXPECT_FALSE(glonass_params::isRepresentable(4091999, glonass::carrierFreqL1OC)); // К10
   EXPECT_TRUE(glonass_params::isRepresentable(20000000, glonass::carrierFreqL1OC)); // К1
}

// Расстройка учитывается вместе с полосой модели: К7 проходит, К8 — нет
TEST(RequestParamsRepresentable, Test3_ResidualFreqCountsTowardBand) {
   const std::int64_t shifted = glonass::carrierFreqL1OC - 1000000;

   EXPECT_TRUE(glonass_params::isRepresentable(20000000, shifted)); // К7: 3 046 000 ≤ 10 000 000
   EXPECT_FALSE(glonass_params::isRepresentable(5000000, shifted)); // К8: 3 046 000 > 2 500 000
   EXPECT_TRUE(glonass_params::isRepresentable(6092000, shifted));  // равенство: 3 046 000 = 3 046 000
}

// Отказ по В.2 — для режимов, выполняющих прогон; разряд отказа unrealizable (422)
TEST(RequestParamsRepresentable, Test4_RequireRejectsAsUnrealizable) {
   const Rejection rejection = rejectionOf([] {
      glonass_params::requireRepresentable(4091999, glonass::carrierFreqL1OC, "fs");
   });

   EXPECT_TRUE(rejection.thrown);
   EXPECT_EQ(rejection.kind,    RejectKind::unrealizable);
   EXPECT_EQ(rejection.field,   "fs");
   EXPECT_EQ(rejection.message, "нарушено условие представимости В.2: |Δf| + B_model > Fs/2");
   EXPECT_NO_THROW(glonass_params::requireRepresentable(4092000, glonass::carrierFreqL1OC, "fs"));
}

// Предусловие Б_L1OC.8: Fs ≥ R_с
TEST(RequestParamsSampleRate, Test1_SymbolRatePrecondition) {
   const Rejection rejection = rejectionOf([] {
      glonass_params::requireSymbolRate(glonass::symbolRateL1OC - 1, "fs");
   });

   EXPECT_TRUE(rejection.thrown);
   EXPECT_EQ(rejection.kind, RejectKind::unrealizable);
   EXPECT_NE(rejection.message.find("Б_L1OC.8"), std::string::npos);
   EXPECT_NO_THROW(glonass_params::requireSymbolRate(glonass::symbolRateL1OC, "fs"));
}

TEST(RequestParamsSampleRate, Test2_PositiveAndStartSample) {
   for (const std::int64_t value : { -1LL, 0LL }) {
      const Rejection rejection = rejectionOf([value] {
         glonass_params::requireSampleRate(value, "fs");
      });

      EXPECT_TRUE(rejection.thrown);
      EXPECT_EQ(rejection.kind, RejectKind::badValue);
   }
   EXPECT_NO_THROW(glonass_params::requireSampleRate(glonass_params::defaultSampleRate, "fs"));

   const Rejection negativeStart = rejectionOf([] {
      glonass_params::requireStartSample(-1, "n0");
   });

   EXPECT_TRUE(negativeStart.thrown);
   EXPECT_EQ(negativeStart.kind,  RejectKind::badValue);
   EXPECT_EQ(negativeStart.field, "n0");
   EXPECT_NO_THROW(glonass_params::requireStartSample(0, "n0"));
}

// Разбор целых и вещественных строгий: значение расходуется целиком
TEST(RequestParamsScalar, Test1_StrictParsing) {
   EXPECT_EQ(glonass_params::parseInteger("20000000", "fs"), 20000000);
   EXPECT_EQ(glonass_params::parseInteger("-5", "n0"),       -5);
   EXPECT_DOUBLE_EQ(glonass_params::parseReal("12.5", "t"), 12.5);

   for (const char* text : { "", "12.5", "5abc", "1e6" }) {
      const Rejection rejection = rejectionOf([text] {
         glonass_params::parseInteger(text, "fs");
      });

      EXPECT_TRUE(rejection.thrown) << text;
      EXPECT_EQ(rejection.kind, RejectKind::badValue) << text;
   }

   for (const char* text : { "", "abc", "1.5.2" }) {
      const Rejection rejection = rejectionOf([text] {
         glonass_params::parseReal(text, "t");
      });

      EXPECT_TRUE(rejection.thrown) << text;
   }
}
