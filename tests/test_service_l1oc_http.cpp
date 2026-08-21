#include "service.h"
#include "service_config.h"
#include "service_version.h"

#include <gtest/gtest.h>
#include <httplib.h>
#include <atomic>
#include <chrono>
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
   EXPECT_EQ(response->status,                           200);
   EXPECT_EQ(response->get_header_value("Content-Type"), "application/json; charset=utf-8");
   EXPECT_EQ(response->body,                             "{\"status\": \"ok\"}");
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

// Точка режима В вводится последующим этапом; до этого путь не обслуживается.
TEST_F(ServiceHttp, Test3_UnknownPathGivesErrorModel) {
   httplib::Client client(localHost, port_);
   const auto response = client.Get("/v1/jobs");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status,                           404);
   EXPECT_EQ(response->get_header_value("Content-Type"), "application/json; charset=utf-8");
   EXPECT_EQ(response->body,
             "{\"error\": \"not_found\", "
             "\"message\": \"путь не обслуживается: /v1/jobs\"}");
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

// ───────────────── режим А — показатели состояния ─────────────────

// пустая строка запроса эквивалентна умолчаниям модуля запуска
TEST_F(ServiceHttp, Test7_StateWithDefaults) {
   httplib::Client client(localHost, port_);
   const auto response = client.Get("/v1/state");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status,                           200);
   EXPECT_EQ(response->get_header_value("Content-Type"), "application/json; charset=utf-8");
   EXPECT_EQ(response->body,
             "{\"n\": 0, \"t\": 0, \"band\": \"L1OC\", \"satelliteCount\": 24, "
             "\"normalizationFactor\": 0.20412414523193154, \"modelBandwidthHz\": 2046000, "
             "\"residualFreqHz\": 0, \"representable\": true, "
             "\"message\": {\"lineIndex\": 0, \"lineType\": \"normal\", "
             "\"convSymbolIndex\": 0, \"lineLength\": 500}}");
}

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
// повторное обращение даёт тот же ответ
TEST_F(ServiceHttp, Test12_StateIsPureFunction) {
   httplib::Client client(localHost, port_);
   const auto first  = client.Get("/v1/state?j=1:8&t=3.5");
   const auto second = client.Get("/v1/state?j=1:8&t=3.5");

   ASSERT_TRUE(first);
   ASSERT_TRUE(second);
   EXPECT_EQ(first->status, 200);
   EXPECT_EQ(first->body,   second->body);
}

// ───────────────────────── режим Б — потоковая выдача ─────────────────────────

// Тело — сырые двоичные отсчёты без заголовка и разделителей; длина задаётся n
TEST_F(ServiceHttp, Test13_StreamCf32DeliversRawSamples) {
   httplib::Client client(localHost, port_);
   const auto response = client.Get("/v1/stream?j=1&n=1024&format=cf32");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status,                                200);
   EXPECT_EQ(response->get_header_value("Content-Type"),      "application/octet-stream");
   EXPECT_EQ(response->get_header_value("Transfer-Encoding"), "chunked");
   EXPECT_EQ(response->body.size(),                           1024U * 8U);

   // I[0] = −1, Q[0] = +0,0 (Д_L1OC.11); порядок байтов прямой
   EXPECT_EQ(response->body.substr(0, 8),                     std::string("\x00\x00\x80\xBF\x00\x00\x00\x00", 8));
}

// Умолчание формата — cs16 (решение 6): 4 байта на отсчёт
TEST_F(ServiceHttp, Test14_StreamDefaultsToCs16) {
   httplib::Client client(localHost, port_);
   const auto response = client.Get("/v1/stream?j=1&n=1024");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status,      200);
   EXPECT_EQ(response->body.size(), 1024U * 4U);
}

// Разбиение на блоки на содержание потока не влияет
TEST_F(ServiceHttp, Test15_StreamContentIndependentOfBlockSamples) {
   httplib::Client client(localHost, port_);
   const auto small = client.Get("/v1/stream?j=1&n=5000&format=cf32&blockSamples=64");
   const auto large = client.Get("/v1/stream?j=1&n=5000&format=cf32&blockSamples=65536");

   ASSERT_TRUE(small);
   ASSERT_TRUE(large);
   EXPECT_EQ(small->status, 200);
   EXPECT_EQ(small->body,   large->body);
}

// Нарушение В.2 отклоняется ДО начала выдачи: код 422 и тело модели ошибок, а не обрезанный
// поток отсчётов (в отличие от режима А, где признак выводится полем при коде 200).
TEST_F(ServiceHttp, Test16_StreamRejectsNonRepresentableBeforeOutput) {
   httplib::Client client(localHost, port_);
   const auto response = client.Get("/v1/stream?j=1&n=1024&fs=4091999");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status,                           422);
   EXPECT_EQ(response->get_header_value("Content-Type"), "application/json; charset=utf-8");
   EXPECT_TRUE(contains(response->body, "\"error\": \"unprocessable\""));
   EXPECT_TRUE(contains(response->body, "\"field\": \"fs\""));
   EXPECT_TRUE(contains(response->body, "В.2"));
}

TEST_F(ServiceHttp, Test17_StreamBadValueGives400) {
   httplib::Client client(localHost, port_);

   for (const char* target : { "/v1/stream?j=1&n=0",
                               "/v1/stream?j=1&n=8&blockSamples=1048577",
                               "/v1/stream?j=1&n=8&format=int8" }) {
      const auto response = client.Get(target);

      ASSERT_TRUE(response) << target;
      EXPECT_EQ(response->status, 400) << target;
      EXPECT_TRUE(contains(response->body, "\"error\": \"bad_request\"")) << target;
   }
}

// ───────────────────── кадры точка /v1/frames ─────────────────────

// Числовые ряды кадра: те же параметры сигнала, что и в прочих режимах
TEST_F(ServiceHttp, Test19_FramePsdGivesSeriesJson) {
   httplib::Client client(localHost, port_);

   client.set_read_timeout(30, 0); // прогон 262 144 отсчётов в сборке Debug
   const auto response = client.Get("/v1/frames/psd?j=1:2");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status,                           200);
   EXPECT_EQ(response->get_header_value("Content-Type"), "application/json; charset=utf-8");
   EXPECT_TRUE(contains(response->body, "\"kind\": \"psd\""));
   EXPECT_TRUE(contains(response->body, "\"points\": 1639"));
   EXPECT_TRUE(contains(response->body, "\"avgDb\""));
}

// Тот же кадр изображением: суффикс .svg выбирает представление
TEST_F(ServiceHttp, Test20_FramePsdGivesSvgImage) {
   httplib::Client client(localHost, port_);

   client.set_read_timeout(30, 0);
   const auto response = client.Get("/v1/frames/psd.svg?j=1:2");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status,                           200);
   EXPECT_EQ(response->get_header_value("Content-Type"), "image/svg+xml; charset=utf-8");
   EXPECT_TRUE(contains(response->body, "viewBox=\"0 0 960 540\""));
   EXPECT_TRUE(contains(response->body, "</svg>"));
}

// Кадры набора, не введённые реализацией, дают 404 по общей модели ошибок.
TEST_F(ServiceHttp, Test21_UnknownFrameKindGivesNotFound) {
   httplib::Client client(localHost, port_);
   const auto response = client.Get("/v1/frames/navline");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status, 404);
   EXPECT_TRUE(contains(response->body, "\"error\": \"not_found\""));
}

// Непригодная конфигурация отклоняется до прогона: кадр требует формирования отсчётов
TEST_F(ServiceHttp, Test22_FrameRejectsBadAndUnrealizableParameters) {
   httplib::Client client(localHost, port_);
   const auto badValue = client.Get("/v1/frames/psd?fs=0");

   ASSERT_TRUE(badValue);
   EXPECT_EQ(badValue->status, 400);
   const auto unrealizable = client.Get("/v1/frames/psd?fs=2000000");

   ASSERT_TRUE(unrealizable);
   EXPECT_EQ(unrealizable->status, 422);
}

// Корреляционные кадры считаются по таблицам ДК: прогона источника нет. Состав из одного НКА
// пар не образует — кадр ВКФ не определён
TEST_F(ServiceHttp, Test23_CorrelationFramesFollowComposition) {
   httplib::Client client(localHost, port_);
   const auto acf = client.Get("/v1/frames/acf?j=7,9");

   ASSERT_TRUE(acf);
   EXPECT_EQ(acf->status, 200);
   EXPECT_TRUE(contains(acf->body, "\"kind\": \"acf\""));
   EXPECT_TRUE(contains(acf->body, "\"satellite\": 7"));
   const auto ccf = client.Get("/v1/frames/ccf.svg?j=7,9");

   ASSERT_TRUE(ccf);
   EXPECT_EQ(ccf->status,                           200);
   EXPECT_EQ(ccf->get_header_value("Content-Type"), "image/svg+xml; charset=utf-8");
   const auto single = client.Get("/v1/frames/ccf?j=7");

   ASSERT_TRUE(single);
   EXPECT_EQ(single->status, 422);
   EXPECT_TRUE(contains(single->body, "\"error\": \"unprocessable\""));
}

TEST_F(ServiceHttp, Test18_StreamLimitGivesUnavailable) {
   ASSERT_EQ(service_->config().maxStreams, 1);
   std::atomic<bool> streaming{ false };
   std::atomic<bool> release{ false };

   std::thread holder([this, &streaming, &release] {
                      httplib::Client client(localHost, port_);

                      client.set_read_timeout(10, 0);

// Поток без предела: n и seconds не заданы
                      client.Get("/v1/stream?j=1&blockSamples=256",
                                 [&streaming, &release](const char*, std::size_t) {
                                 streaming.store(true);
                                 return !release.load(); // приём прекращается по сигналу — обрыв получателем
         });
      });

   for (int attempt = 0; (attempt < 2000) && !streaming.load(); ++attempt) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
   }
   EXPECT_TRUE(streaming.load());

   httplib::Client client(localHost, port_);
   const auto rejected = client.Get("/v1/stream?j=1&n=8");

   ASSERT_TRUE(rejected);
   EXPECT_EQ(rejected->status, 503);
   EXPECT_TRUE(contains(rejected->body, "\"error\": \"unavailable\""));

   // Пул не исчерпан: служебная точка отвечает во время потока
   const auto health = client.Get("/healthz");

   ASSERT_TRUE(health);
   EXPECT_EQ(health->status, 200);

   release.store(true);
   holder.join();

   // Место освобождено разрушением ответа: следующий поток принимается
   const auto accepted = client.Get("/v1/stream?j=1&n=8&format=cf32");

   ASSERT_TRUE(accepted);
   EXPECT_EQ(accepted->status,      200);
   EXPECT_EQ(accepted->body.size(), 8U * 8U);
}
