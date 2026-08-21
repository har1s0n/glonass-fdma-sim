#include "tcp_stream_session.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <exception>
#include <span>
#include <thread>
#include <utility>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "service.h"
#include "service_config.h"
#include "stream_slot.h"

namespace glonass_service {
namespace {
constexpr int listenBacklog = 1;

// Доставку SIGPIPE при записи в разорванное соединение подавляют разными средствами:
// в Linux — признак вызова, в macOS — настройка сокета (см. configureConnection).
#ifdef MSG_NOSIGNAL
constexpr int sendFlags = MSG_NOSIGNAL;
#else // ifdef MSG_NOSIGNAL
constexpr int sendFlags = 0;
#endif // ifdef MSG_NOSIGNAL

void closeSocket(int& descriptor) noexcept {
   if (descriptor >= 0) {
      ::close(descriptor);
      descriptor = -1;
   }
}

// Слушающий сокет на заданном порту
int openListener(int port) {
   const int listener = ::socket(AF_INET, SOCK_STREAM, 0);

   if (listener < 0) {
      return -1;
   }
   int enable = 1;

   // Без SO_REUSEADDR повторное занятие порта в пределах TIME_WAIT отказывает, и порт
   // диапазона оказывался бы недоступен ещё минуту после закрытия сеанса.
   ::setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
   sockaddr_in address{};

   address.sin_family      = AF_INET;
   address.sin_port        = htons(static_cast<std::uint16_t> (port));
   address.sin_addr.s_addr = ::inet_addr(bindHost); // 0.0.0.0: порты наружу не публикуются

   if ((::bind(listener, reinterpret_cast<const sockaddr*> (&address), sizeof(address)) < 0) ||
       (::listen(listener, listenBacklog) < 0)) {
      int failed = listener;

      closeSocket(failed);
      return -1;
   }
   return listener;
}

void configureConnection(int connection) {
#ifdef SO_NOSIGPIPE
   int enable = 1;

   ::setsockopt(connection, SOL_SOCKET, SO_NOSIGPIPE, &enable, sizeof(enable));
#endif // ifdef SO_NOSIGPIPE
   timeval timeout{};

   timeout.tv_sec = writeTimeoutSeconds;
   ::setsockopt(connection, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
}
} // namespace

class TcpSession {
public:

   TcpSession(std::string sessionId, int port, int listener, const StreamRequest& request,
              std::unique_ptr<StreamSlot> slot, int acceptTimeoutSeconds,
              std::uint64_t requestId)
      : sessionId_(std::move(sessionId)), port_(port), listener_(listener), request_(request),
      slot_(std::move(slot)), acceptTimeoutSeconds_(acceptTimeoutSeconds),
      requestId_(requestId) {
      if (::pipe(wakeup_) != 0) {
         wakeup_[0] = -1;
         wakeup_[1] = -1;
      }
      worker_ = std::thread([this] {
            run();
         });
   }

   TcpSession(const TcpSession&)            = delete;
   TcpSession &operator=(const TcpSession&) = delete;

   ~TcpSession() {
      requestStop();

      if (worker_.joinable()) {
         worker_.join();
      }
      closeSocket(wakeup_[0]);
      closeSocket(wakeup_[1]);
   }

   // Прерывание сеанса: Вызывается из потока обработки запроса.
   void requestStop() noexcept {
      stopRequested_.store(true);

      if (wakeup_[1] >= 0) {
         const char token = 1;

         (void)::write(wakeup_[1], &token, 1); // однобайтовая запись в канал не блокирует
      }
      const std::lock_guard<std::mutex> lock(socketMutex_);

      // Закрытие дескрипторов выполняет только поток сеанса и только под этим замком,
      // поэтому здесь дескриптор либо действителен, либо уже -1.
      if (connection_ >= 0) {
         ::shutdown(connection_, SHUT_RDWR);
      }
   }

   const std::string &id() const noexcept {
      return sessionId_;
   }

   int port() const noexcept {
      return port_;
   }

   bool finished() const noexcept {
      return finished_.load();
   }

   // Сеанс закрыт точкой DELETE: по идентификатору более не отыскивается.
   void markRemoved() noexcept {
      removed_ = true;
   }

   bool removed() const noexcept {
      return removed_;
   }

private:

   void run();
   int  acceptOne();
   bool sendBlock(int                            connection,
                  std::span<const unsigned char> block);

   std::string sessionId_;
   int port_       = 0;
   int listener_   = -1;
   int connection_ = -1;
   int wakeup_[2]  = { -1, -1 };
   StreamRequest request_;
   std::unique_ptr<StreamSlot> slot_; // место в пределе; возвращается разрушением сеанса
   int acceptTimeoutSeconds_ = 0;
   std::uint64_t requestId_  = 0;
   std::mutex socketMutex_;           // доступ к connection_ из потока запроса и потока сеанса
   std::atomic<bool> stopRequested_{ false };
   std::atomic<bool> finished_{ false };
   bool removed_ = false;
   std::thread worker_;
};

void TcpSession::run() {
   const int connection = acceptOne();

   {
      const std::lock_guard<std::mutex> lock(socketMutex_);

      closeSocket(listener_); // порт возвращается сразу: подключение на сеанс одно
      connection_ = connection;
   }

   if (connection < 0) {
      logLine("INFO", requestId_,
              "сеанс " + sessionId_ + ": подключения не было, порт " + std::to_string(port_)
              + (stopRequested_.load() ? "; сеанс закрыт" : "; ожидание истекло"));
      finished_.store(true);
      return;
   }
   configureConnection(connection);
   logLine("INFO", requestId_,
           "сеанс " + sessionId_ + ": получатель подключён, порт " + std::to_string(port_));
   std::int64_t emitted = 0;
   bool completed       = false;

   try {
      // Объект-источник создаётся здесь и существует всё время потока
      StreamSession session(request_);

      while (!stopRequested_.load()) {
         const std::span<const unsigned char> block = session.nextBlock();

         if (block.empty()) { // заданная длительность выдана полностью
            completed = true;
            break;
         }

         if (!sendBlock(connection, block)) {
            break;
         }
      }
      emitted = session.samplesEmitted();
   } catch (const std::exception& error) {
      logLine("ERROR", requestId_, "сеанс " + sessionId_ + ": " + error.what());
   } catch (...) {
      logLine("ERROR", requestId_, "сеанс " + sessionId_ + ": неопознанное исключение");
   }
   {
      const std::lock_guard<std::mutex> lock(socketMutex_);

      if (connection_ >= 0) {
         ::shutdown(connection_, SHUT_RDWR);
         closeSocket(connection_);
      }
   }
   logLine(completed ? "INFO" : "WARN", requestId_,
           "сеанс " + sessionId_ + " завершён: отсчётов " + std::to_string(emitted)
           + (completed ? "" : "; выдача прервана"));
   finished_.store(true);
}

int TcpSession::acceptOne() {
   const auto deadline = std::chrono::steady_clock::now()
                         + std::chrono::seconds(acceptTimeoutSeconds_);

   while (!stopRequested_.load()) {
      const auto left = std::chrono::duration_cast<std::chrono::milliseconds> (
         deadline - std::chrono::steady_clock::now()).count();

      if (left <= 0) {
         return -1;
      }
      pollfd watched[2]{};

      watched[0].fd     = listener_;
      watched[0].events = POLLIN;
      watched[1].fd     = wakeup_[0];
      watched[1].events = POLLIN;
      const int ready = ::poll(watched, 2, static_cast<int> (left));

      if (ready < 0) {
         if (errno == EINTR) {
            continue;
         }
         return -1;
      }

      if (ready == 0) { // предел ожидания подключения истёк
         return -1;
      }

      if ((watched[1].revents & POLLIN) != 0) { // сеанс закрыт до подключения
         return -1;
      }

      if ((watched[0].revents & POLLIN) != 0) {
         const int connection = ::accept(listener_, nullptr, nullptr);

         if ((connection < 0) && (errno == EINTR)) {
            continue;
         }
         return connection;
      }
      return -1; // отказ слушающего сокета
   }
   return -1;
}

// Запись блока целиком; false — соединение разорвано, предел ожидания записи истёк либо
// сеанс закрыт. Дробление блока средой передачи штатно: блок — единица конвейера, а не
// формата, разделителей в потоке нет.
bool TcpSession::sendBlock(int connection, std::span<const unsigned char> block) {
   std::size_t sent = 0;

   while (sent < block.size()) {
      const ssize_t written = ::send(connection, block.data() + sent, block.size() - sent,
                                     sendFlags);

      if (written > 0) {
         sent += static_cast<std::size_t> (written);
         continue;
      }

      if ((written < 0) && (errno == EINTR)) {
         continue;
      }
      return false;
   }
   return true;
}

TcpStreamRegistry::TcpStreamRegistry(std::atomic<int>& streamCounter, int maxStreams,
                                     int portFirst, int portLast, int acceptTimeoutSeconds)
   : streamCounter_(streamCounter), maxStreams_(maxStreams), portFirst_(portFirst),
   portLast_(portLast), acceptTimeoutSeconds_(acceptTimeoutSeconds) {}

TcpStreamRegistry::~TcpStreamRegistry() {
   closeAll();
}

TcpOpenResult TcpStreamRegistry::open(const StreamRequest& request, std::uint64_t requestId) {
   const std::lock_guard<std::mutex> lock(mutex_);

   reapFinished();

   auto slot = std::make_unique<StreamSlot> (streamCounter_, maxStreams_);

   if (!slot->acquired()) {
      return TcpOpenResult{ TcpOpenStatus::limitReached, "", 0 };
   }
   int listener = -1;
   int port     = 0;

   // Первый занимаемый порт по возрастанию. Занятый своим сеансом либо посторонним
   // процессом отсеивается отказом bind
   for (int candidate = portFirst_; candidate <= portLast_; ++candidate) {
      listener = openListener(candidate);

      if (listener >= 0) {
         port = candidate;
         break;
      }
   }

   if (listener < 0) { // место в пределе возвращается разрушением slot
      return TcpOpenResult{ TcpOpenStatus::noFreePort, "", 0 };
   }
   const std::string sessionId = nextSessionId();

   sessions_.push_back(std::make_unique<TcpSession> (sessionId, port, listener, request,
                                                     std::move(slot), acceptTimeoutSeconds_,
                                                     requestId));
   return TcpOpenResult{ TcpOpenStatus::opened, sessionId, port };
}

bool TcpStreamRegistry::close(const std::string& sessionId) {
   const std::lock_guard<std::mutex> lock(mutex_);

   reapFinished();

   for (const auto& session : sessions_) {
      if (!session->removed() && (session->id() == sessionId)) {
         session->markRemoved();
         session->requestStop(); // ожидание завершения потока сеанса откладывается до пожинания
         return true;
      }
   }
   return false;
}

void TcpStreamRegistry::closeAll() {
   const std::lock_guard<std::mutex> lock(mutex_);

   for (const auto& session : sessions_) {
      session->requestStop();
   }
   sessions_.clear(); // разрушение каждого сеанса дожидается завершения его потока
}

std::size_t TcpStreamRegistry::sessionCount() {
   const std::lock_guard<std::mutex> lock(mutex_);

   reapFinished();
   return sessions_.size();
}

void TcpStreamRegistry::reapFinished() {
   const auto finished = std::remove_if(sessions_.begin(), sessions_.end(),
                                        [](const std::unique_ptr<TcpSession>& session) {
         return session->finished();
      });

   sessions_.erase(finished, sessions_.end());
}

std::string TcpStreamRegistry::nextSessionId() {
   char buffer[24];

   std::snprintf(buffer, sizeof(buffer), "s-%04llx",
                 static_cast<unsigned long long> (++idCounter_));
   return buffer;
}
} // namespace glonass_service
