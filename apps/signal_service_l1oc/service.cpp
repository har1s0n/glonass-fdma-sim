#include "service.h"

#include <chrono>
#include <ctime>
#include <exception>
#include <iostream>
#include <mutex>
#include <string>

#include "error_response.h"
#include "json_writer.h"
#include "service_version.h"

namespace glonass_service {
namespace {
constexpr const char* contentTypeJson = "application/json";

// Идентификатор запроса ведётся на поток: соединение обслуживается одним потоком пула,
// поэтому значение доступно и обработчику, и записи в журнал.
thread_local std::uint64_t threadRequestId = 0;

// Отметка времени UTC
std::string timestampUtc() {
   const auto now            = std::chrono::system_clock::now();
   const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
   std::tm parts{};

   gmtime_r(&seconds, &parts);
   char buffer[32];

   std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &parts);
   return buffer;
}

void respondWithError(httplib::Response& response, const ErrorResponse& error) {
   response.status = error.status;
   response.set_content(error.body(), contentTypeJson);
}
} // namespace

std::uint64_t currentRequestId() {
   return threadRequestId;
}

void logLine(const std::string& level, std::uint64_t requestId, const std::string& message) {
   static std::mutex outputMutex;
   const std::lock_guard<std::mutex> lock(outputMutex);

   std::cout << timestampUtc() << ' ' << level << ' ' << requestId << ' ' << message << '\n'
             << std::flush;
}

Service::Service(const ServiceConfig& config)
   : config_(config) {
   registerHandlers();
   registerRoutes();
}

void Service::registerHandlers() {
   // Перед маршрутизацией: присвоение идентификатора запроса и проверка состояния сервиса
   server_.set_pre_routing_handler([this](const httplib::Request&, httplib::Response& response) {
         threadRequestId = requestCounter_.fetch_add(1) + 1;

         if (shuttingDown_.load()) {
            respondWithError(response, unavailable("сервис завершает работу"));
            return httplib::Server::HandlerResponse::Handled;
         }
         return httplib::Server::HandlerResponse::Unhandled;
      });

   server_.set_error_handler([](const httplib::Request& request, httplib::Response& response) {
         if (!response.body.empty()) {
            return;
         }
         ErrorResponse error;

         error.status  = response.status;
         error.error   = errorSlugForStatus(response.status);
         error.message = (response.status == 404)
                      ? ("путь не обслуживается: " + request.path)
                      : "запрос не обработан";
         respondWithError(response, error);
      });

   server_.set_exception_handler([](const httplib::Request&, httplib::Response& response,
                                    std::exception_ptr exception) {
         std::string reason = "неопознанное исключение";

         try {
            std::rethrow_exception(exception);
         } catch (const std::exception& error) {
            reason = error.what();
         } catch (...) {
            // причина недоступна: сохраняется значение по умолчанию
         }
         logLine("ERROR", currentRequestId(), "исключение при обработке: " + reason);
         respondWithError(response, internalError(reason));
      });

   server_.set_logger([](const httplib::Request& request, const httplib::Response& response) {
         logLine("INFO", currentRequestId(),
                 request.method + ' ' + request.target + " -> " + std::to_string(response.status));
      });
}

void Service::registerRoutes() {
   server_.Get("/healthz", [](const httplib::Request&, httplib::Response& response) {
         JsonObject json;

         json.addString("status", "ok");
         response.set_content(json.str(), contentTypeJson);
      });

   // Сведения о сервисе
   server_.Get("/v1/info", [](const httplib::Request&, httplib::Response& response) {
         JsonObject json;

         json.addString("service",    serviceName);
         json.addString("version",    serviceVersion);
         json.addString("api",        apiVersion);
         json.addString("band",       band);
         json.addString("icdProfile", icdProfile);
         response.set_content(json.str(), contentTypeJson);
      });
}

bool Service::bindToPort(const std::string& host, int port) {
   return server_.bind_to_port(host, port);
}

int Service::bindToAnyPort(const std::string& host) {
   const int port = server_.bind_to_any_port(host);

   return port < 0 ? 0 : port;
}

bool Service::listenAfterBind() {
   return server_.listen_after_bind();
}

void Service::beginShutdown() {
   shuttingDown_.store(true);
}

void Service::stop() {
   server_.stop();
}

bool Service::isRunning() const {
   return server_.is_running();
}

void Service::waitUntilReady() const {
   server_.wait_until_ready();
}

int runHealthcheck(const std::string& host, int port) {
   httplib::Client client(host, port);

   client.set_connection_timeout(2, 0);
   client.set_read_timeout(2, 0);
   const auto result = client.Get("/healthz");

   return (result && (result->status == 200)) ? 0 : 1;
}
} // namespace glonass_service
