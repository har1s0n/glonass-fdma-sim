#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "service.h"
#include "service_config.h"
#include "service_version.h"

namespace {
volatile std::sig_atomic_t stopRequested = 0;

// Обработчик сигнала ограничен записью флага
extern "C" void onTerminationSignal(int /*signalNumber*/) {
   stopRequested = 1;
}
} // namespace

int main(int argc, char** argv) try {
   bool healthcheck = false;

   for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];

      if (key == "--healthcheck") {
         healthcheck = true;
      } else if ((key == "--help") ||
                 (key == "-h")) {
         std::cout << glonass_service::serviceName << " [--healthcheck]\n"
                                                      "  --healthcheck  проба собственной точки /healthz; код возврата 0 — норма\n"
                                                      "Параметры развёртывания — переменные окружения:\n"
                                                      "  " << glonass_service::envPort     << "      порт интерфейса HTTP\n"
                                                                                              "  " << glonass_service::envMaxJobs  <<
            "  одновременно выполняемых заданий\n"
            "  "
                   << glonass_service::envWorkDir  << "  каталог временных артефактов\n"
                                            "  "
                   << glonass_service::envTcpPorts << " диапазон портов потоковых сеансов\n";
         return 0;
      } else {
         throw std::runtime_error("неизвестный аргумент: " + key);
      }
   }
   const glonass_service::ServiceConfig config =
      glonass_service::parseServiceConfig(glonass_service::readEnvironment());

   if (healthcheck) {
      return glonass_service::runHealthcheck("127.0.0.1", config.port);
   }
   glonass_service::Service service(config);

   if (!service.bindToPort(glonass_service::bindHost, config.port)) {
      throw std::runtime_error("не удалось занять порт " + std::to_string(config.port));
   }
   std::signal(SIGTERM, onTerminationSignal);
   std::signal(SIGINT,  onTerminationSignal);

   std::atomic<bool> listenFinished{ false };
   std::thread watcher([&service, &listenFinished] {
                       while (!listenFinished.load()) {
                          if (stopRequested != 0) {
                             glonass_service::logLine("INFO", 0, "получен сигнал завершения");
                             service.beginShutdown(); // обращения получают 503 до закрытия приёма соединений
                             service.stop();
                             return;
                          }
                          std::this_thread::sleep_for(std::chrono::milliseconds(100));
                       }
      });

   glonass_service::logLine("INFO", 0,
                            std::string("запуск ") + glonass_service::serviceName
                            + " версии " + glonass_service::serviceVersion
                            + ", тракт " + glonass_service::band
                            + ", порт " + std::to_string(config.port)
                            + ", заданий " + std::to_string(config.maxJobs)
                            + ", каталог " + config.workDir
                            + ", порты сеансов " + std::to_string(config.tcpPortFirst)
                            + "-" + std::to_string(config.tcpPortLast));
   const bool listened = service.listenAfterBind();

   listenFinished.store(true);
   watcher.join();
   glonass_service::logLine("INFO", 0, "останов завершён");
   return listened ? 0 : 1;
}
catch (const std::exception& e) {
   std::cerr << "Ошибка: " << e.what() << "\n";
   return 1;
}
