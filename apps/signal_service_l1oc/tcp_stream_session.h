#ifndef SERVICE_TCP_STREAM_SESSION_H
#define SERVICE_TCP_STREAM_SESSION_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "stream_session.h"

// Режим Б — потоковый сеанс по сырому TCP (точки POST /v1/stream/tcp и
// DELETE /v1/stream/tcp/{sessionId})
//
// Порядок сеанса: POST открывает слушающий сокет на порту из диапазона SIGNAL_TCP_PORTS и
// сразу отвечает; прогон начинается при подключении получателя.
//
// Подключение принимается одно: слушающий сокет закрывается сразу после него, а завершение
// прогона — по исчерпанию длительности, обрыву получателем, закрытию точкой DELETE либо
// истечению ожидания подключения — закрывает сеанс окончательно.
//
// Ввод-вывод блокирующий, поэтому каждый сеанс ведётся собственным потоком вне
// пула httplib
namespace glonass_service {
enum class TcpOpenStatus {
   opened,       // сеанс открыт: порт занят, ожидается подключение получателя
   limitReached, // исчерпан предел одновременно открытых потоков (SIGNAL_MAX_STREAMS)
   noFreePort    // в диапазоне SIGNAL_TCP_PORTS свободного порта нет
};

struct TcpOpenResult {
   TcpOpenStatus status = TcpOpenStatus::noFreePort;
   std::string   sessionId;
   int           port = 0;
};

class TcpSession; // владеет сокетами и потоком сеанса; определён в единице трансляции

class TcpStreamRegistry {
public:

   TcpStreamRegistry(std::atomic<int>& streamCounter,
                     int               maxStreams,
                     int               portFirst,
                     int               portLast,
                     int               acceptTimeoutSeconds);
   ~TcpStreamRegistry();

   TcpStreamRegistry(const TcpStreamRegistry&)            = delete;
   TcpStreamRegistry &operator=(const TcpStreamRegistry&) = delete;

   // Открытие сеанса. Место в пределе занимается здесь, а не при подключении получателя:
   // иначе сеансы, открытые в ожидании, начали бы выдачу разом и предел был бы превышен.
   TcpOpenResult open(const StreamRequest& request,
                      std::uint64_t        requestId);

   // Закрытие по идентификатору. Выдача обрывается немедленно
   // false — сеанс неизвестен: не открывался, уже закрыт либо завершился сам.
   bool        close(const std::string& sessionId);

   // Останов сервиса: обрыв всех сеансов с ожиданием завершения их потоков.
   void        closeAll();

   // Число сеансов, ведомых реестром: открытые и закрытые, но ещё не пожатые.
   std::size_t sessionCount();

private:

   // возвращает порты и места в пределе.
   // Вызывается при захваченном mutex_.
   void        reapFinished();
   std::string nextSessionId();

   std::mutex mutex_;
   std::vector<std::unique_ptr<TcpSession> > sessions_;
   std::atomic<int>& streamCounter_;
   int maxStreams_;
   int portFirst_;
   int portLast_;
   int acceptTimeoutSeconds_;
   std::uint64_t idCounter_ = 0;
};
} // namespace glonass_service

#endif // SERVICE_TCP_STREAM_SESSION_H
