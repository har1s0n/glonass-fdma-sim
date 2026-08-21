#ifndef SERVICE_STREAM_SLOT_H
#define SERVICE_STREAM_SLOT_H

#include <atomic>

// Предел одновременно открытых потоков режима Б (SIGNAL_MAX_STREAMS)
//
// Предел общий для обеих точек режима Б: поток HTTP (GET /v1/stream) и потоковый сеанс по
// сырому TCP (POST /v1/stream/tcp) нагружают генерацию одинаково, поэтому счётчик один.
namespace glonass_service {
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
} // namespace glonass_service

#endif // SERVICE_STREAM_SLOT_H
