#ifndef SERVICE_H
#define SERVICE_H

#include <atomic>
#include <cstdint>
#include <string>

#include "httplib.h"
#include "service_config.h"

// каркас микросервиса цифровой модели сигнала L1OC

namespace glonass_service {
class Service {
public:

   explicit Service(const ServiceConfig& config);
   bool                 bindToPort(const std::string& host,
                                   int                port);
   int                  bindToAnyPort(const std::string& host);

   // Приём соединений; управление возвращается после stop().
   bool                 listenAfterBind();

   // Перевод в состояние завершения: приём соединений продолжается, обращения получают 503
   void                 beginShutdown();
   void                 stop();
   bool                 isRunning() const;
   void                 waitUntilReady() const;

   const ServiceConfig &config() const noexcept {
      return config_;
   }

private:

   void registerHandlers();
   void registerRoutes();

private:

   ServiceConfig config_;
   httplib::Server server_;
   std::atomic<bool> shuttingDown_{ false };
   std::atomic<std::uint64_t> requestCounter_{ 0 };
};

// Режим --healthcheck исполняемого файла: обращение к собственной точке /healthz.
int runHealthcheck(const std::string& host,
                   int                port);

// Идентификатор запроса, обрабатываемого текущим потоком; 0 — вне обработки запроса.
// Присваивается перед маршрутизацией, выводится в строке журнала.
std::uint64_t currentRequestId();

// Строка журнала в стандартный вывод: отметка времени, уровень, идентификатор запроса, сообщение
void          logLine(const std::string& level,
                      std::uint64_t      requestId,
                      const std::string& message);
} // namespace glonass_service

#endif // SERVICE_H
