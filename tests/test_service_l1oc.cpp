#include "error_response.h"
#include "json_writer.h"
#include "service_config.h"

#include <gtest/gtest.h>
#include <map>
#include <stdexcept>
#include <string>

namespace {
using glonass_service::ErrorResponse;
using glonass_service::JsonObject;
using glonass_service::parseServiceConfig;
using glonass_service::ServiceConfig;

std::map<std::string, std::string> environmentOf(
   std::initializer_list<std::pair<const std::string, std::string> > values) {
   return std::map<std::string, std::string> (values);
}
} // namespace

// ───────────────────────────── сборка JSON ─────────────────────────────

TEST(ServiceJsonWriter, Test1_EscapeSpecialCharacters) {
   EXPECT_EQ(glonass_service::jsonEscape("кавычка \" внутри"), "кавычка \\\" внутри");
   EXPECT_EQ(glonass_service::jsonEscape("косая \\ черта"),    "косая \\\\ черта");
   EXPECT_EQ(glonass_service::jsonEscape("строка\nстрока"),    "строка\\nстрока");
   EXPECT_EQ(glonass_service::jsonEscape("столбец\tстолбец"),  "столбец\\tстолбец");
   EXPECT_EQ(glonass_service::jsonEscape(std::string("\x01")), "\\u0001");
}

// Байты ≥ 0x80 переносятся без изменения: тело кодируется в UTF-8.
TEST(ServiceJsonWriter, Test2_Utf8PassesThrough) {
   const std::string text = "суммарный сигнал";

   EXPECT_EQ(glonass_service::jsonEscape(text), text);
}

TEST(ServiceJsonWriter, Test3_ObjectKeepsInsertionOrder) {
   JsonObject json;

   json.addString("service", "signal-service-l1oc").addString("api", "v1");
   EXPECT_EQ(json.str(), "{\"service\": \"signal-service-l1oc\", \"api\": \"v1\"}");
}

TEST(ServiceJsonWriter, Test4_ValueTypes) {
   EXPECT_EQ(JsonObject().str(),                                "{}");
   EXPECT_EQ(JsonObject().addInt("n", 250000000).str(),         "{\"n\": 250000000}");
   EXPECT_EQ(JsonObject().addInt("n0", -1).str(),               "{\"n0\": -1}");
   EXPECT_EQ(JsonObject().addBool("representable", true).str(), "{\"representable\": true}");
   EXPECT_EQ(JsonObject().addBool("representable", false).str(),
             "{\"representable\": false}");
   EXPECT_EQ(JsonObject().addDouble("eta", 0.5).str(),          "{\"eta\": 0.5}");
   EXPECT_EQ(JsonObject().addDouble("amp", 2.0).str(),          "{\"amp\": 2}");
}

TEST(ServiceJsonWriter, Test5_NestedRawFragment) {
   JsonObject line;

   line.addInt("index", 6).addString("type", "normal");
   JsonObject json;

   json.addInt("n", 1).addRaw("line", line.str());
   EXPECT_EQ(json.str(), "{\"n\": 1, \"line\": {\"index\": 6, \"type\": \"normal\"}}");
}

// ───────────────────────────── модель ошибок ─────────────────────────────

// Поле field выводится только когда ошибка отнесена к конкретному параметру запроса.
TEST(ServiceErrorModel, Test6_BodyWithAndWithoutField) {
   const ErrorResponse withField = glonass_service::badRequest("fs", "значение не разбирается");

   EXPECT_EQ(withField.body(),
             "{\"error\": \"bad_request\", \"field\": \"fs\", "
             "\"message\": \"значение не разбирается\"}");

   const ErrorResponse withoutField = glonass_service::notFound("путь не обслуживается: /v1/x");

   EXPECT_EQ(withoutField.body(),
             "{\"error\": \"not_found\", \"message\": \"путь не обслуживается: /v1/x\"}");
}

// Коды состояния — по таблице «Модель ошибок» контракта.
TEST(ServiceErrorModel, Test7_StatusOfBuilders) {
   EXPECT_EQ(glonass_service::badRequest("fs", "").status,    400);
   EXPECT_EQ(glonass_service::notFound("").status,            404);
   EXPECT_EQ(glonass_service::conflict("").status,            409);
   EXPECT_EQ(glonass_service::unprocessable("fs", "").status, 422);
   EXPECT_EQ(glonass_service::tooManyRequests("").status,     429);
   EXPECT_EQ(glonass_service::internalError("").status,       500);
   EXPECT_EQ(glonass_service::unavailable("").status,         503);
}

TEST(ServiceErrorModel, Test8_SlugForStatus) {
   EXPECT_EQ(glonass_service::errorSlugForStatus(400), "bad_request");
   EXPECT_EQ(glonass_service::errorSlugForStatus(404), "not_found");
   EXPECT_EQ(glonass_service::errorSlugForStatus(409), "conflict");
   EXPECT_EQ(glonass_service::errorSlugForStatus(422), "unprocessable");
   EXPECT_EQ(glonass_service::errorSlugForStatus(429), "too_many_requests");
   EXPECT_EQ(glonass_service::errorSlugForStatus(500), "internal");
   EXPECT_EQ(glonass_service::errorSlugForStatus(503), "unavailable");
   EXPECT_EQ(glonass_service::errorSlugForStatus(418), "internal"); // код вне таблицы
}

// ───────────────────────────── конфигурация развёртывания ─────────────────────────────

TEST(ServiceConfiguration, Test9_DefaultsOnEmptyEnvironment) {
   const ServiceConfig config = parseServiceConfig({});

   EXPECT_EQ(config.port,         8080);
   EXPECT_EQ(config.maxJobs,      1);
   EXPECT_EQ(config.workDir,      "/tmp/signal");
   EXPECT_EQ(config.tcpPortFirst, 30070);
   EXPECT_EQ(config.tcpPortLast,  30079);
}

TEST(ServiceConfiguration, Test10_FullOverride) {
   const ServiceConfig config = parseServiceConfig(environmentOf({
      { glonass_service::envPort,     "9090" },
      { glonass_service::envMaxJobs,  "3" },
      { glonass_service::envWorkDir,  "/var/tmp/signal" },
      { glonass_service::envTcpPorts, "31000-31003" },
   }));

   EXPECT_EQ(config.port,         9090);
   EXPECT_EQ(config.maxJobs,      3);
   EXPECT_EQ(config.workDir,      "/var/tmp/signal");
   EXPECT_EQ(config.tcpPortFirst, 31000);
   EXPECT_EQ(config.tcpPortLast,  31003);
}

// Диапазон из одного порта допустим: границы включительны.
TEST(ServiceConfiguration, Test11_SinglePortRange) {
   const ServiceConfig config = parseServiceConfig(environmentOf({
      { glonass_service::envTcpPorts, "30070-30070" },
   }));

   EXPECT_EQ(config.tcpPortFirst, 30070);
   EXPECT_EQ(config.tcpPortLast,  30070);
}

TEST(ServiceConfiguration, Test12_RejectedValues) {
   EXPECT_THROW(parseServiceConfig(environmentOf({ { glonass_service::envPort, "0" } })),
                std::runtime_error);
   EXPECT_THROW(parseServiceConfig(environmentOf({ { glonass_service::envPort, "70000" } })),
                std::runtime_error);
   EXPECT_THROW(parseServiceConfig(environmentOf({ { glonass_service::envPort, "8080x" } })),
                std::runtime_error);
   EXPECT_THROW(parseServiceConfig(environmentOf({ { glonass_service::envPort, "" } })),
                std::runtime_error);
   EXPECT_THROW(parseServiceConfig(environmentOf({ { glonass_service::envMaxJobs, "0" } })),
                std::runtime_error);
   EXPECT_THROW(parseServiceConfig(environmentOf({ { glonass_service::envWorkDir, "" } })),
                std::runtime_error);
   EXPECT_THROW(parseServiceConfig(environmentOf({ { glonass_service::envTcpPorts, "30070" } })),
                std::runtime_error);
   EXPECT_THROW(parseServiceConfig(environmentOf({
      { glonass_service::envTcpPorts, "30079-30070" },
   })), std::runtime_error);
   EXPECT_THROW(parseServiceConfig(environmentOf({
      { glonass_service::envTcpPorts, "30070-70000" },
   })), std::runtime_error);
}

// Сообщение об ошибке называет переменную: иначе причина отказа запуска контейнера неясна.
TEST(ServiceConfiguration, Test13_MessageNamesVariable) {
   try {
      parseServiceConfig(environmentOf({ { glonass_service::envMaxJobs, "-1" } }));
      FAIL() << "ожидалось исключение";
   } catch (const std::runtime_error& error) {
      const std::string message = error.what();

      EXPECT_NE(message.find(glonass_service::envMaxJobs), std::string::npos);
   }
}
