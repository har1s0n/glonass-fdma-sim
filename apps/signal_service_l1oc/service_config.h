#ifndef SERVICE_CONFIG_H
#define SERVICE_CONFIG_H

#include <cstdlib>
#include <map>
#include <stdexcept>
#include <string>

// Параметры развёртывания из переменных окружения
// Параметры сигнала здесь не задаются: они передаются строкой запроса и разбираются отдельно.
namespace glonass_service {
inline constexpr const char* envPort       = "SIGNAL_PORT";
inline constexpr const char* envMaxJobs    = "SIGNAL_MAX_JOBS";
inline constexpr const char* envMaxStreams = "SIGNAL_MAX_STREAMS";
inline constexpr const char* envWorkDir    = "SIGNAL_WORK_DIR";
inline constexpr const char* envTcpPorts   = "SIGNAL_TCP_PORTS";

// Адрес привязки: порты наружу не публикуются, сервис доступен только из сети комплекса,
// поэтому привязка выполняется ко всем интерфейсам контейнера
inline constexpr const char* bindHost = "0.0.0.0";

// Предел ожидания готовности сокета к записи
inline constexpr int writeTimeoutSeconds = 150;

struct ServiceConfig {
   int         port         = 8080;          // SIGNAL_PORT [порт интерфейса HTTP]
   int         maxJobs      = 1;             // SIGNAL_MAX_JOBS [одновременно выполняемых заданий]
   int         maxStreams   = 1;             // SIGNAL_MAX_STREAMS [одновременно открытых потоков]
   std::string workDir      = "/tmp/signal"; // SIGNAL_WORK_DIR [временные артефакты заданий]
   int         tcpPortFirst = 30070;         // SIGNAL_TCP_PORTS [диапазон портов
   int         tcpPortLast  = 30079;         // потоковых сеансов]

   // Предел ожидания подключения получателя к порту потокового сеанса по сырому TCP.
   // Слушающий сокет открывается на POST /v1/stream/tcp; если подключения нет в течение
   // предела, сеанс закрывается сам и возвращает порт вместе с местом в пределе — иначе
   // забытый DELETE безвозвратно занимал бы порт диапазона
   int tcpAcceptTimeout = 150; // ожидание подключения получателя, с
};

namespace detail {
inline int parseIntStrict(const std::string& text, const std::string& variable) {
   if (text.empty()) {
      throw std::runtime_error(variable + ": пустое значение");
   }
   std::size_t consumed = 0;
   long long   value    = 0;

   try {
      value = std::stoll(text, &consumed);
   } catch (const std::exception&) {
      throw std::runtime_error(variable + ": значение не является целым числом: " + text);
   }
   if (consumed != text.size()) {
      throw std::runtime_error(variable + ": значение не является целым числом: " + text);
   }

   if ((value < -2147483648LL) || (value > 2147483647LL)) {
      throw std::runtime_error(variable + ": значение вне диапазона: " + text);
   }
   return static_cast<int> (value);
}

inline void requirePortRange(int value, const std::string& variable) {
   if ((value < 1) || (value > 65535)) {
      throw std::runtime_error(variable + ": номер порта вне диапазона 1…65535: "
                               + std::to_string(value));
   }
}

// Диапазон портов задаётся как "первый-последний", границы включительно.
inline void parsePortRange(const std::string& text, const std::string& variable,
                           int& first, int& last) {
   const std::size_t separator = text.find('-');

   if (separator == std::string::npos) {
      throw std::runtime_error(variable + ": ожидается диапазон вида 30070-30079: " + text);
   }
   first = parseIntStrict(text.substr(0, separator), variable);
   last  = parseIntStrict(text.substr(separator + 1), variable);
   requirePortRange(first, variable);
   requirePortRange(last,  variable);

   if (first > last) {
      throw std::runtime_error(variable + ": начало диапазона больше конца: " + text);
   }
}
} // namespace detail

inline ServiceConfig parseServiceConfig(const std::map<std::string, std::string>& environment) {
   ServiceConfig config;
   auto valueOf = [&environment](const char* name, const std::string** out) {
                     const auto found = environment.find(name);

                     if (found == environment.end()) {
                        return false;
                     }
                     *out = &found->second;
                     return true;
                  };
   const std::string* value = nullptr;

   if (valueOf(envPort, &value)) {
      config.port = detail::parseIntStrict(*value, envPort);
      detail::requirePortRange(config.port, envPort);
   }

   if (valueOf(envMaxJobs, &value)) {
      config.maxJobs = detail::parseIntStrict(*value, envMaxJobs);

      if (config.maxJobs < 1) {
         throw std::runtime_error(std::string(envMaxJobs) + ": требуется значение ≥ 1: "
                                  + std::to_string(config.maxJobs));
      }
   }

   // Поток режима Б удерживает поток пула на всё время выдачи; без предела бесконечные потоки
   // исчерпали бы пул и служебные точки перестали бы отвечать.
   if (valueOf(envMaxStreams, &value)) {
      config.maxStreams = detail::parseIntStrict(*value, envMaxStreams);

      if (config.maxStreams < 1) {
         throw std::runtime_error(std::string(envMaxStreams) + ": требуется значение ≥ 1: "
                                  + std::to_string(config.maxStreams));
      }
   }

   if (valueOf(envWorkDir, &value)) {
      if (value->empty()) {
         throw std::runtime_error(std::string(envWorkDir) + ": пустое значение");
      }
      config.workDir = *value;
   }

   if (valueOf(envTcpPorts, &value)) {
      detail::parsePortRange(*value, envTcpPorts, config.tcpPortFirst, config.tcpPortLast);
   }
   return config;
}

inline std::map<std::string, std::string> readEnvironment() {
   std::map<std::string, std::string> environment;

   for (const char* name : { envPort, envMaxJobs, envMaxStreams, envWorkDir, envTcpPorts }) {
      const char* value = std::getenv(name);

      if (value != nullptr) {
         environment.emplace(name, value);
      }
   }
   return environment;
}
} // namespace glonass_service

#endif // SERVICE_CONFIG_H
