#ifndef SERVICE_ERROR_RESPONSE_H
#define SERVICE_ERROR_RESPONSE_H

#include <string>

#include "json_writer.h"

// единая модель ошибок сервиса
//
// Тело ответа единообразно: {"error": "код", "field": "имя", "message": "пояснение"}
// Поле field выводится только когда ошибка относится к конкретному параметру запроса.
// Коды состояния и условия их выдачи:
//   400 — параметр отсутствует, не разбирается либо вне допустимого диапазона;
//   404 — неизвестный идентификатор задания или сеанса (а также неизвестный путь);
//   409 — операция несовместима с текущим состоянием ресурса;
//   422 — параметры формально корректны, конфигурация нереализуема;
//   429 — очередь заданий заполнена;
//   500 — внутренняя ошибка;
//   503 — сервис запускается или завершает работу.
namespace glonass_service {
struct ErrorResponse {
   int         status = 500;
   std::string error;
   std::string field; // имя параметра; пустое в тело не выводится
   std::string message;

   std::string body() const {
      JsonObject json;

      json.addString("error", error);

      if (!field.empty()) {
         json.addString("field", field);
      }
      json.addString("message", message);
      return json.str();
   }
};

inline std::string errorSlugForStatus(int status) {
   switch (status) {
     case 400: return "bad_request";

     case 404: return "not_found";

     case 409: return "conflict";

     case 422: return "unprocessable";

     case 429: return "too_many_requests";

     case 503: return "unavailable";

     default:  return "internal";
   }
}

inline ErrorResponse badRequest(const std::string& field, const std::string& message) {
   return ErrorResponse{ 400, "bad_request", field, message };
}

inline ErrorResponse notFound(const std::string& message) {
   return ErrorResponse{ 404, "not_found", "", message };
}

inline ErrorResponse conflict(const std::string& message) {
   return ErrorResponse{ 409, "conflict", "", message };
}

inline ErrorResponse unprocessable(const std::string& field, const std::string& message) {
   return ErrorResponse{ 422, "unprocessable", field, message };
}

inline ErrorResponse tooManyRequests(const std::string& message) {
   return ErrorResponse{ 429, "too_many_requests", "", message };
}

inline ErrorResponse internalError(const std::string& message) {
   return ErrorResponse{ 500, "internal", "", message };
}

inline ErrorResponse unavailable(const std::string& message) {
   return ErrorResponse{ 503, "unavailable", "", message };
}
} // namespace glonass_service

#endif // SERVICE_ERROR_RESPONSE_H
