#include "service.h"
#include "service_config.h"
#include "stream_session.h"

#include <gtest/gtest.h>
#include <httplib.h>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

// Режим Б микросервиса L1OC — потоковый сеанс по сырому TCP
//

namespace {
using glonass_service::parseStreamRequest;
using glonass_service::Service;
using glonass_service::ServiceConfig;
using glonass_service::StreamRequest;
using glonass_service::StreamSession;

constexpr const char* localHost = "127.0.0.1";

// Диапазоны проверок отделены от рабочего (30070–30079): прогон не должен зависеть от
// того, работает ли на машине сам сервис.
constexpr int testPortFirst = 30190;
constexpr int testPortLast  = 30191;
constexpr int timeoutPort   = 30192; // проверка истечения ожидания подключения
constexpr int occupiedPort  = 30193; // проверка исчерпания диапазона

using Parameters = std::vector<std::pair<std::string, std::string> >;

bool contains(const std::string& text, const std::string& fragment) {
   return text.find(fragment) != std::string::npos;
}

std::string queryOf(const Parameters& parameters) {
   std::string query;

   for (const auto& [key, value] : parameters) {
      query += query.empty() ? '?' : '&';
      query += key + '=' + value;
   }
   return query;
}

// Тот же разбор, что выполняет точка: эталон и запрос не расходятся в умолчаниях.
StreamRequest requestOf(const Parameters& parameters) {
   httplib::Request request;

   for (const auto& [key, value] : parameters) {
      request.params.emplace(key, value);
   }
   return parseStreamRequest(request);
}

// Полный поток заданной длительности в байтах выходного формата
std::vector<unsigned char> referenceStream(const StreamRequest& request) {
   StreamSession session(request);
   std::vector<unsigned char> bytes;

   for (std::span<const unsigned char> block = session.nextBlock(); !block.empty();
        block = session.nextBlock()) {
      bytes.insert(bytes.end(), block.begin(), block.end());
   }
   return bytes;
}

// Значение числового поля тела JSON; -1 — поле отсутствует
std::int64_t intFieldOf(const std::string& body, const std::string& key) {
   const std::string marker = "\"" + key + "\": ";
   const std::size_t start  = body.find(marker);

   if (start == std::string::npos) {
      return -1;
   }
   return std::stoll(body.substr(start + marker.size()));
}

std::string stringFieldOf(const std::string& body, const std::string& key) {
   const std::string marker = "\"" + key + "\": \"";
   const std::size_t start  = body.find(marker);

   if (start == std::string::npos) {
      return {};
   }
   const std::size_t from = start + marker.size();

   return body.substr(from, body.find('"', from) - from);
}

// Подключение получателя; -1 — порт не обслуживается. Предел ожидания приёма страхует
// прогон от зависания при дефекте выдачи.
int connectTo(int port) {
   const int receiver = ::socket(AF_INET, SOCK_STREAM, 0);

   if (receiver < 0) {
      return -1;
   }
   sockaddr_in address{};

   address.sin_family      = AF_INET;
   address.sin_port        = htons(static_cast<std::uint16_t> (port));
   address.sin_addr.s_addr = ::inet_addr(localHost);

   if (::connect(receiver, reinterpret_cast<const sockaddr*> (&address), sizeof(address)) < 0) {
      ::close(receiver);
      return -1;
   }
   timeval timeout{};

   timeout.tv_sec = 10;
   ::setsockopt(receiver, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
   return receiver;
}

// Слушающий сокет проверки: занимает порт диапазона посторонним по отношению к сервису
// процессом-владельцем.
int occupyPort(int port) {
   const int listener = ::socket(AF_INET, SOCK_STREAM, 0);

   if (listener < 0) {
      return -1;
   }
   sockaddr_in address{};

   address.sin_family      = AF_INET;
   address.sin_port        = htons(static_cast<std::uint16_t> (port));
   address.sin_addr.s_addr = htonl(INADDR_ANY);

   if ((::bind(listener, reinterpret_cast<const sockaddr*> (&address), sizeof(address)) < 0) ||
       (::listen(listener, 1) < 0)) {
      ::close(listener);
      return -1;
   }
   return listener;
}

// Приём не более limit байт; возвращается по концу потока либо по достижении предела
std::vector<unsigned char> receive(int receiver, std::size_t limit) {
   std::vector<unsigned char> received;
   unsigned char chunk[4096];

   while (received.size() < limit) {
      const std::size_t want  = std::min(sizeof(chunk), limit - received.size());
      const ssize_t     taken = ::recv(receiver, chunk, want, 0);

      if (taken <= 0) {
         break;
      }
      received.insert(received.end(), chunk, chunk + taken);
   }
   return received;
}

// Приём до конца потока; false — предел не достигнут и поток не закрыт (истёк SO_RCVTIMEO)
bool drainUntilClosed(int receiver, std::size_t limit) {
   std::size_t   received = 0;
   unsigned char chunk[4096];

   while (received < limit) {
      const ssize_t taken = ::recv(receiver, chunk, sizeof(chunk), 0);

      if (taken == 0) { // конец потока: сеанс закрыт
         return true;
      }

      if (taken < 0) {
         return errno != EAGAIN && errno != EWOULDBLOCK; // разрыв соединения — тоже закрытие
      }
      received += static_cast<std::size_t> (taken);
   }
   return false;
}

class TcpStream : public ::testing::Test {
protected:

   // Конфигурация задаётся тестом: диапазон портов и предел ожидания подключения
   void start(const ServiceConfig& config) {
      service_ = std::make_unique<Service> (config);
      port_    = service_->bindToAnyPort(localHost);
      ASSERT_GT(port_, 0);
      listener_ = std::thread([this] {
            service_->listenAfterBind();
         });
      service_->waitUntilReady();
   }

   ServiceConfig testConfig() const {
      ServiceConfig config;

      config.tcpPortFirst = testPortFirst;
      config.tcpPortLast  = testPortLast;
      return config; // maxStreams = 1, tcpAcceptTimeout = 60
   }

   void TearDown() override {
      if (service_) {
         service_->stop();
         listener_.join();
      }
   }

   std::unique_ptr<Service> service_;
   std::thread listener_;
   int port_ = 0;
};
} // namespace

// Открытие сеанса: порт из диапазона, идентификатор и параметры прогона в ответе (§ 5.2)
TEST_F(TcpStream, Test1_PostOpensSessionOnRangePort) {
   start(testConfig());
   httplib::Client client(localHost, port_);
   const auto response = client.Post("/v1/stream/tcp?j=1&n=512&format=cf32&blockSamples=128");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status,                           201);
   EXPECT_EQ(response->get_header_value("Content-Type"), "application/json; charset=utf-8");
   const std::string sessionId = stringFieldOf(response->body, "sessionId");

   EXPECT_FALSE(sessionId.empty());
   EXPECT_EQ(response->get_header_value("Location"), "/v1/stream/tcp/" + sessionId);
   const std::int64_t sessionPort = intFieldOf(response->body, "port");

   EXPECT_GE(sessionPort, testPortFirst);
   EXPECT_LE(sessionPort, testPortLast);
   EXPECT_EQ(stringFieldOf(response->body, "format"),    "cf32");
   EXPECT_EQ(intFieldOf(response->body, "blockSamples"), 128);
}

// Получателю выдаются те же байты, что даёт StreamSession на тех же параметрах;
// по исчерпании заданной длительности сеанс закрывает соединение.
TEST_F(TcpStream, Test2_ReceiverGetsSameBytesAsStreamSession) {
   start(testConfig());
   const Parameters parameters{
      { "j", "1:3" }, { "n", "512" }, { "format", "cf32" }, { "blockSamples", "128" }
   };
   const std::vector<unsigned char> expected = referenceStream(requestOf(parameters));

   ASSERT_EQ(expected.size(), 512U * 8U); // 512 отсчётов CF32
   httplib::Client client(localHost, port_);
   const auto response = client.Post("/v1/stream/tcp" + queryOf(parameters));

   ASSERT_TRUE(response);
   ASSERT_EQ(response->status, 201);
   const int receiver = connectTo(static_cast<int> (intFieldOf(response->body, "port")));

   ASSERT_GE(receiver, 0);
   const std::vector<unsigned char> received = receive(receiver, expected.size() + 64);

   ::close(receiver);
   EXPECT_EQ(received.size(), expected.size()); // предел приёма не достигнут: поток закрыт сам
   EXPECT_TRUE(received == expected);
}

// Закрытие точкой DELETE обрывает выдачу немедленно, не дожидаясь границы блока
TEST_F(TcpStream, Test3_DeleteInterruptsActiveStream) {
   start(testConfig());
   httplib::Client client(localHost, port_);

   // Поток без предела: n и seconds не заданы (§ 5.2)
   const auto opened = client.Post("/v1/stream/tcp?j=1&blockSamples=256&format=cf32");

   ASSERT_TRUE(opened);
   ASSERT_EQ(opened->status, 201);
   const int receiver = connectTo(static_cast<int> (intFieldOf(opened->body, "port")));

   ASSERT_GE(receiver, 0);
   EXPECT_EQ(receive(receiver, 256U * 8U).size(), 256U * 8U); // выдача идёт

   const std::string sessionId = stringFieldOf(opened->body, "sessionId");
   const auto closed           = client.Delete("/v1/stream/tcp/" + sessionId);

   ASSERT_TRUE(closed);
   EXPECT_EQ(closed->status, 200);
   EXPECT_TRUE(contains(closed->body, "\"status\": \"closed\""));

   // Выдача прекращается: приём завершается концом потока, а не пределом ожидания
   EXPECT_TRUE(drainUntilClosed(receiver, 64U * 1024U * 1024U));
   ::close(receiver);
}

// Обрыв немедленный, а не по границе блока: получатель, прекративший чтение, держит выдачу
// в обратном давлении, и ожидание границы отложило бы закрытие до предела writeTimeoutSeconds
TEST_F(TcpStream, Test4_DeleteInterruptsBlockedWrite) {
   start(testConfig());
   httplib::Client client(localHost, port_);

   // Длительность заведомо больше буферов сокета: выдача упирается в неснимаемое окно приёма
   constexpr std::int64_t sampleCount = 2000000; // 16 МБ в формате CF32
   const auto opened                  = client.Post("/v1/stream/tcp?j=1&format=cf32&blockSamples=65536&n="
                                                    + std::to_string(sampleCount));

   ASSERT_TRUE(opened);
   ASSERT_EQ(opened->status, 201);
   const int receiver = connectTo(static_cast<int> (intFieldOf(opened->body, "port")));

   ASSERT_GE(receiver, 0);
   std::this_thread::sleep_for(std::chrono::milliseconds(300)); // получатель не читает
   const auto closed = client.Delete("/v1/stream/tcp/" + stringFieldOf(opened->body, "sessionId"));

   ASSERT_TRUE(closed);
   EXPECT_EQ(closed->status, 200);

   // Место в пределе освобождается, пока получатель по-прежнему не читает: поток сеанса
   // разворачивается сразу, а не по возобновлении чтения и не по пределу ожидания записи.
   bool released = false;

   for (int attempt = 0; (attempt < 200) && !released; ++attempt) {
      const auto next = client.Post("/v1/stream/tcp?j=1&n=512");

      ASSERT_TRUE(next);
      released = next->status == 201;

      if (!released) {
         std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
   }
   EXPECT_TRUE(released);

   // Принятое — только осевшее в буферах: поток оборван, а не выдан целиком
   const std::size_t whole                   = static_cast<std::size_t> (sampleCount) * 8U;
   const std::vector<unsigned char> received = receive(receiver, whole);

   ::close(receiver);
   EXPECT_LT(received.size(), whole);
}

// Идентификатор неизвестен — 404 по общей модели ошибок
TEST_F(TcpStream, Test5_DeleteUnknownSessionGivesNotFound) {
   start(testConfig());
   httplib::Client client(localHost, port_);
   const auto response = client.Delete("/v1/stream/tcp/s-ffff");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status, 404);
   EXPECT_TRUE(contains(response->body, "\"error\": \"not_found\""));
}

// Порт и место в пределе возвращаются: после закрытия сеанс открывается снова
TEST_F(TcpStream, Test6_PortAndSlotReturnedAfterDelete) {
   start(testConfig());
   httplib::Client client(localHost, port_);
   const auto first = client.Post("/v1/stream/tcp?j=1&n=512");

   ASSERT_TRUE(first);
   ASSERT_EQ(first->status, 201);
   const std::int64_t firstPort = intFieldOf(first->body, "port");
   const auto closed            = client.Delete("/v1/stream/tcp/" + stringFieldOf(first->body, "sessionId"));

   ASSERT_TRUE(closed);
   ASSERT_EQ(closed->status, 200);

   // Освобождение асинхронно: поток сеанса завершается сам, пожинание выполняется
   // очередным обращением к точкам сеансов.
   httplib::Result second(nullptr, httplib::Error::Unknown);

   for (int attempt = 0; attempt < 500; ++attempt) {
      second = client.Post("/v1/stream/tcp?j=1&n=512");
      ASSERT_TRUE(second);

      if (second->status == 201) {
         break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
   }
   ASSERT_EQ(second->status, 201);
   EXPECT_EQ(intFieldOf(second->body, "port"), firstPort); // тот же порт занимается снова
}

// Предел SIGNAL_MAX_STREAMS общий с потоком HTTP: сверх него сеанс не открывается,
// служебные точки при этом отвечают
TEST_F(TcpStream, Test7_LimitRejectsSecondSession) {
   start(testConfig());
   ASSERT_EQ(service_->config().maxStreams, 1);
   httplib::Client client(localHost, port_);
   const auto opened = client.Post("/v1/stream/tcp?j=1&n=512");

   ASSERT_TRUE(opened);
   ASSERT_EQ(opened->status, 201);
   const auto rejected = client.Post("/v1/stream/tcp?j=1&n=512");

   ASSERT_TRUE(rejected);
   EXPECT_EQ(rejected->status, 503);
   EXPECT_TRUE(contains(rejected->body, "\"error\": \"unavailable\""));
   const auto health = client.Get("/healthz");

   ASSERT_TRUE(health);
   EXPECT_EQ(health->status, 200);
}

// Подключения не было: по истечении предела ожидания сеанс закрывается сам и возвращает
// порт вместе с местом в пределе
TEST_F(TcpStream, Test8_AcceptTimeoutReleasesPort) {
   ServiceConfig config = testConfig();

   config.tcpPortFirst     = timeoutPort;
   config.tcpPortLast      = timeoutPort;
   config.tcpAcceptTimeout = 1;
   start(config);
   httplib::Client client(localHost, port_);
   const auto opened = client.Post("/v1/stream/tcp?j=1&n=512");

   ASSERT_TRUE(opened);
   ASSERT_EQ(opened->status,                   201);
   ASSERT_EQ(intFieldOf(opened->body, "port"), timeoutPort);
   std::this_thread::sleep_for(std::chrono::milliseconds(1500));

   // Порт более не обслуживается, а единственное место в пределе свободно
   EXPECT_EQ(connectTo(timeoutPort), -1);
   const auto reopened = client.Post("/v1/stream/tcp?j=1&n=512");

   ASSERT_TRUE(reopened);
   EXPECT_EQ(reopened->status,                   201);
   EXPECT_EQ(intFieldOf(reopened->body, "port"), timeoutPort);
}

// Непригодная конфигурация отклоняется до занятия порта: 400 — значение вне диапазона,
// 422 — условие представимости В.2 не выполнено
TEST_F(TcpStream, Test9_BadParametersRejectedBeforeListener) {
   start(testConfig());
   httplib::Client client(localHost, port_);
   const auto badValue = client.Post("/v1/stream/tcp?fs=0");

   ASSERT_TRUE(badValue);
   EXPECT_EQ(badValue->status, 400);
   EXPECT_TRUE(contains(badValue->body, "\"error\": \"bad_request\""));
   const auto unrealizable = client.Post("/v1/stream/tcp?fs=2000000");

   ASSERT_TRUE(unrealizable);
   EXPECT_EQ(unrealizable->status, 422);
   EXPECT_TRUE(contains(unrealizable->body, "\"error\": \"unprocessable\""));

   // Порт диапазона не занят: слушающий сокет не открывался
   EXPECT_EQ(connectTo(testPortFirst), -1);
}

// Свободного порта в диапазоне нет: сеанс не открывается, место в пределе возвращается
TEST_F(TcpStream, Test10_NoFreePortGivesUnavailable) {
   const int occupied = occupyPort(occupiedPort);

   ASSERT_GE(occupied, 0);
   ServiceConfig config = testConfig();

   config.tcpPortFirst = occupiedPort;
   config.tcpPortLast  = occupiedPort;
   start(config);
   httplib::Client client(localHost, port_);
   const auto response = client.Post("/v1/stream/tcp?j=1&n=512");

   ASSERT_TRUE(response);
   EXPECT_EQ(response->status, 503);
   EXPECT_TRUE(contains(response->body, "\"error\": \"unavailable\""));
   EXPECT_TRUE(contains(response->body, "свободный порт"));
   ::close(occupied);

   // Место в пределе не удержано отказом: после освобождения порта сеанс открывается
   const auto opened = client.Post("/v1/stream/tcp?j=1&n=512");

   ASSERT_TRUE(opened);
   EXPECT_EQ(opened->status, 201);
}

// Останов сервиса закрывает сеансы: потоки сеансов не переживают останов
TEST_F(TcpStream, Test11_StopClosesActiveSession) {
   start(testConfig());
   httplib::Client client(localHost, port_);
   const auto opened = client.Post("/v1/stream/tcp?j=1&blockSamples=256&format=cf32");

   ASSERT_TRUE(opened);
   ASSERT_EQ(opened->status, 201);
   const int receiver = connectTo(static_cast<int> (intFieldOf(opened->body, "port")));

   ASSERT_GE(receiver, 0);
   EXPECT_EQ(receive(receiver, 256U * 8U).size(), 256U * 8U);
   service_->stop();
   listener_.join();
   service_.reset(); // TearDown повторного останова не выполняет
   EXPECT_TRUE(drainUntilClosed(receiver, 64U * 1024U * 1024U));
   ::close(receiver);
}
