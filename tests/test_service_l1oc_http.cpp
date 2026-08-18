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
   EXPECT_TRUE(contains(response->get_header_value("Content-Type"), "application/json"));
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

// Точки данных вводятся последующими этапами; до этого путь не обслуживается.
TEST_F(ServiceHttp, Test3_UnknownPathGivesErrorModel) {
   httplib::Client client(localHost, port_);
   const auto response = client.Get("/v1/state");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status, 404);
   EXPECT_TRUE(contains(response->get_header_value("Content-Type"), "application/json"));
   EXPECT_TRUE(contains(response->body, "\"error\": \"not_found\""));
   EXPECT_TRUE(contains(response->body, "/v1/state"));
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
