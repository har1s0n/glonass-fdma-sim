#include "service.h"
#include "service_config.h"
#include "service_version.h"

#include <gtest/gtest.h>
#include <httplib.h>
#include <memory>
#include <string>
#include <thread>

namespace {
constexpr const char* localHost = "127.0.0.1";

// Порт 1 привилегированный и не обслуживается: обращение к нему заведомо отклоняется.
constexpr int unusedPort = 1;

bool contains(const std::string& text, const std::string& fragment) {
   return text.find(fragment) != std::string::npos;
}

class ServiceHttp : public ::testing::Test {
protected:

   void SetUp() override {
      const glonass_service::ServiceConfig config; // умолчания; порт задаётся привязкой

      service_ = std::make_unique<glonass_service::Service> (config);
      port_    = service_->bindToAnyPort(localHost);
      ASSERT_GT(port_, 0);
      listener_ = std::thread([this] {
            service_->listenAfterBind();
         });
      service_->waitUntilReady();
   }

   void TearDown() override {
      service_->stop();
      listener_.join();
   }

   std::unique_ptr<glonass_service::Service> service_;
   std::thread listener_;
   int port_ = 0;
};
} // namespace

TEST_F(ServiceHttp, Test1_HealthzReportsOk) {
   httplib::Client client(localHost, port_);
   const auto response = client.Get("/healthz");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status, 200);
   EXPECT_EQ(response->get_header_value("Content-Type"), "application/json; charset=utf-8");
   EXPECT_EQ(response->body, "{\"status\": \"ok\"}");
}

TEST_F(ServiceHttp, Test2_InfoReportsServiceAndIcdProfile) {
   httplib::Client client(localHost, port_);
   const auto response = client.Get("/v1/info");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status, 200);
   EXPECT_TRUE(contains(response->body, glonass_service::serviceName));
   EXPECT_TRUE(contains(response->body, "\"version\": \""
                        + std::string(glonass_service::serviceVersion) + "\""));
   EXPECT_TRUE(contains(response->body, "\"api\": \"v1\""));
   EXPECT_TRUE(contains(response->body, "\"band\": \"L1OC\""));
   EXPECT_TRUE(contains(response->body, glonass_service::icdProfile));
}

// Точки режимов Б и В вводятся последующими этапами; до этого путь не обслуживается.
TEST_F(ServiceHttp, Test3_UnknownPathGivesErrorModel) {
   httplib::Client client(localHost, port_);
   const auto response = client.Get("/v1/stream");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status, 404);
   EXPECT_EQ(response->get_header_value("Content-Type"), "application/json; charset=utf-8");
   EXPECT_EQ(response->body,
             "{\"error\": \"not_found\", "
             "\"message\": \"путь не обслуживается: /v1/stream\"}");
}

// Состояние завершения: приём соединений ещё открыт, обращения получают 503.
TEST_F(ServiceHttp, Test4_ShutdownStateGivesUnavailable) {
   service_->beginShutdown();
   httplib::Client client(localHost, port_);
   const auto response = client.Get("/healthz");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status, 503);
   EXPECT_TRUE(contains(response->body, "\"error\": \"unavailable\""));
}

// Режим --healthcheck: код возврата процесса для директивы HEALTHCHECK образа.
TEST_F(ServiceHttp, Test5_HealthcheckReturnCode) {
   EXPECT_EQ(glonass_service::runHealthcheck(localHost, port_),      0);
   EXPECT_EQ(glonass_service::runHealthcheck(localHost, unusedPort), 1);
}

// В состоянии завершения проба неработоспособна: код возврата 1.
TEST_F(ServiceHttp, Test6_HealthcheckFailsWhileShuttingDown) {
   service_->beginShutdown();
   EXPECT_EQ(glonass_service::runHealthcheck(localHost, port_), 1);
}

// ───────────────── режим А — показатели состояния (§ 5.1 контракта) ─────────────────

// Умолчания § 4 контракта: пустая строка запроса эквивалентна умолчаниям модуля запуска
TEST_F(ServiceHttp, Test7_StateWithDefaults) {
   httplib::Client client(localHost, port_);
   const auto response = client.Get("/v1/state");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status, 200);
   EXPECT_EQ(response->get_header_value("Content-Type"), "application/json; charset=utf-8");
   EXPECT_EQ(response->body,
             "{\"n\": 0, \"t\": 0, \"band\": \"L1OC\", \"satelliteCount\": 24, "
             "\"normalizationFactor\": 0.20412414523193154, \"modelBandwidthHz\": 2046000, "
             "\"residualFreqHz\": 0, \"representable\": true, "
             "\"message\": {\"lineIndex\": 0, \"lineType\": \"normal\", "
             "\"convSymbolIndex\": 0, \"lineLength\": 500}}");
}

// Опорная конфигурация К1 расчёта docs/raschet_l1oc/gate_l1oc_state.py
TEST_F(ServiceHttp, Test8_StateReferenceConfiguration) {
   httplib::Client client(localHost, port_);
   const auto response = client.Get("/v1/state?j=1:24&fs=20000000&n0=0&t=12.5");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status, 200);
   EXPECT_EQ(response->body,
             "{\"n\": 250000000, \"t\": 12.5, \"band\": \"L1OC\", \"satelliteCount\": 24, "
             "\"normalizationFactor\": 0.20412414523193154, \"modelBandwidthHz\": 2046000, "
             "\"residualFreqHz\": 0, \"representable\": true, "
             "\"message\": {\"lineIndex\": 6, \"lineType\": \"normal\", "
             "\"convSymbolIndex\": 125, \"lineLength\": 500}}");
}

// Невыполненное условие В.2 не является отказом точки показателей: прочие поля сохраняют
// смысл, признак выводится полем representable
TEST_F(ServiceHttp, Test9_StateReportsNonRepresentableWithCode200) {
   httplib::Client client(localHost, port_);
   const auto response = client.Get("/v1/state?fs=4091999&j=1");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status, 200);
   EXPECT_TRUE(contains(response->body, "\"representable\": false"));
   EXPECT_TRUE(contains(response->body, "\"modelBandwidthHz\": 2046000"));
}

// Значение вне допустимого диапазона — 400 с указанием параметра
TEST_F(ServiceHttp, Test10_StateBadValueGives400) {
   httplib::Client client(localHost, port_);
   const auto response = client.Get("/v1/state?j=0");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status, 400);
   EXPECT_TRUE(contains(response->body, "\"error\": \"bad_request\""));
   EXPECT_TRUE(contains(response->body, "\"field\": \"j\""));
}

// Значения формально корректны, конфигурация нереализуема — 422
TEST_F(ServiceHttp, Test11_StateUnrealizableGives422) {
   httplib::Client client(localHost, port_);
   const auto response = client.Get("/v1/state?j=1&amp=0");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status, 422);
   EXPECT_TRUE(contains(response->body, "\"error\": \"unprocessable\""));
   EXPECT_TRUE(contains(response->body, "\"field\": \"amp\""));
}

// Показатели вычисляются аналитически: точка без побочных эффектов и без состояния,
// повторное обращение даёт тот же ответ (§ 3, решение 2)
TEST_F(ServiceHttp, Test12_StateIsPureFunction) {
   httplib::Client client(localHost, port_);
   const auto first  = client.Get("/v1/state?j=1:8&t=3.5");
   const auto second = client.Get("/v1/state?j=1:8&t=3.5");

   ASSERT_TRUE(first);
   ASSERT_TRUE(second);
   EXPECT_EQ(first->status, 200);
   EXPECT_EQ(first->body, second->body);
}
