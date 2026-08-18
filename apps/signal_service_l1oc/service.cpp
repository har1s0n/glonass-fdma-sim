#include "service.h"

#include <chrono>
#include <ctime>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <span>
#include <string>

#include "error_response.h"
#include "json_writer.h"
#include "service_version.h"
#include "state_metrics.h"
#include "stream_session.h"

namespace glonass_service {
namespace {
constexpr const char* contentTypeJson        = "application/json; charset=utf-8";
constexpr const char* contentTypeOctetStream = "application/octet-stream";

thread_local std::uint64_t threadRequestId = 0;

// Место в пределе одновременно открытых потоков режима Б. Занимается на время жизни объекта;
// объект живёт в замыкании поставщика содержимого и разрушается вместе с ответом — в том числе
// при обрыве соединения получателем.
class StreamSlot {
public:

   StreamSlot(std::atomic<int>& counter, int limit)
      : counter_(counter), acquired_(counter.fetch_add(1) < limit) {
      if (!acquired_) {
         counter_.fetch_sub(1);
      }
   }

   StreamSlot(const StreamSlot&)            = delete;
   StreamSlot &operator=(const StreamSlot&) = delete;

   ~StreamSlot() {
      if (acquired_) {
         counter_.fetch_sub(1);
      }
   }

   bool acquired() const noexcept {
      return acquired_;
   }

private:

   std::atomic<int>& counter_;
   bool acquired_;
};

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

// Разряд отказа разбора параметров → код состояния HTTP: badValue — значение не разбирается
// либо вне допустимого диапазона (400); unrealizable — значения формально корректны,
// конфигурация нереализуема (422).
ErrorResponse fromParamError(const glonass_params::ParamError& error) {
   return (error.kind() == glonass_params::RejectKind::badValue)
          ? badRequest(error.field(), error.what())
          : unprocessable(error.field(), error.what());
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
   // Ожидание готовности сокета к записи — предел обратного давления режима Б (service_config.h)
   server_.set_write_timeout(writeTimeoutSeconds, 0);
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

   // Режим А — показатели состояния. Ядро модели не запускается:
   // показатели выводятся аналитически из конфигурации и модельного времени
   server_.Get("/v1/state", [](const httplib::Request& request, httplib::Response& response) {
         try {
            const StateRequest parsed = parseStateRequest(request);

            response.set_content(stateMetricsJson(computeStateMetrics(parsed)), contentTypeJson);
         } catch (const glonass_params::ParamError& error) {
            respondWithError(response, fromParamError(error));
         }
      });

   // Режим Б — потоковая выдача отсчётов
   server_.Get("/v1/stream", [this](const httplib::Request& request, httplib::Response& response) {
         try {
            const StreamRequest parsed = parseStreamRequest(request);
            auto slot                  = std::make_shared<StreamSlot> (activeStreams_, config_.maxStreams);

            if (!slot->acquired()) {
               respondWithError(response,
                                unavailable("предел одновременно открытых потоков исчерпан: "
                                            + std::to_string(config_.maxStreams)));
               return;
            }
            auto session                  = std::make_shared<StreamSession> (parsed);
            const std::uint64_t requestId = currentRequestId();

            response.set_chunked_content_provider(
               contentTypeOctetStream,
               [session](std::size_t /*offset*/, httplib::DataSink& sink) {
               const std::span<const unsigned char> block = session->nextBlock();

               if (block.empty()) { // заданная длительность выдана полностью
                  sink.done();
                  return true;
               }

               // Запись блокирует выдачу, пока получатель не освободит окно приёма: темп
               // задаётся самым медленным звеном штатным управлением потоком TCP.
               return sink.write(reinterpret_cast<const char*> (block.data()), block.size());
            },

               // Освобождающий вызов деструктора ответа: сессия и место в пределе живут ровно
               // до конца потока — при исчерпании длительности, обрыве и останове сервиса.
               [session, slot, requestId](bool success) {
               logLine(success ? "INFO" : "WARN", requestId,
                       "поток завершён: отсчётов "
                       + std::to_string(session->samplesEmitted())
                       + (success ? "" : "; выдача прервана"));
            });
         } catch (const glonass_params::ParamError& error) {
            respondWithError(response, fromParamError(error));
         }
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
