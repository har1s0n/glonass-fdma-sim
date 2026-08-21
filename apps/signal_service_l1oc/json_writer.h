#ifndef SERVICE_JSON_WRITER_H
#define SERVICE_JSON_WRITER_H

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace glonass_service {
// Экранирование строки для тела JSON: кавычка, обратная косая черта и управляющие символы
// U+0000…U+001F. Байты ≥ 0x80 переносятся без изменения, тело кодируется в UTF-8.
inline std::string jsonEscape(const std::string& text) {
   std::string out;

   out.reserve(text.size());

   for (const char symbol : text) {
      const unsigned char code = static_cast<unsigned char> (symbol);

      switch (symbol) {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\b': out += "\\b";  break;
        case '\f': out += "\\f";  break;
        case '\n': out += "\\n";  break;
        case '\r': out += "\\r";  break;
        case '\t': out += "\\t";  break;
        default:

           if (code < 0x20U) {
              char buffer[7];
              std::snprintf(buffer, sizeof(buffer), "\\u%04X", static_cast<unsigned> (code));
              out += buffer;
           } else {
              out += symbol;
           }
           break;
      }
   }
   return out;
}

// Массив чисел с заданным форматом элемента (например "%.3f") — фрагмент для addRaw
inline std::string jsonNumberArray(const std::vector<double>& values, const char* pattern) {
   std::string out = "[";
   char buffer[48];

   for (std::size_t i = 0; i < values.size(); ++i) {
      if (i > 0) {
         out += ", ";
      }
      std::snprintf(buffer, sizeof(buffer), pattern, values[i]);
      out += buffer;
   }
   return out + "]";
}

// Массив целых — фрагмент для addRaw
template<typename Integer>
inline std::string jsonIntegerArray(const std::vector<Integer>& values) {
   std::string out = "[";

   for (std::size_t i = 0; i < values.size(); ++i) {
      if (i > 0) {
         out += ", ";
      }
      out += std::to_string(values[i]);
   }
   return out + "]";
}

class JsonObject {
public:

   JsonObject &addString(const std::string& key, const std::string& value) {
      appendKey(key);
      body_ += '"';
      body_ += jsonEscape(value);
      body_ += '"';
      return *this;
   }

   JsonObject &addInt(const std::string& key, std::int64_t value) {
      appendKey(key);
      body_ += std::to_string(value);
      return *this;
   }

   // %.17g — то же представление, что в сопроводительном JSON записи I/Q: round-trip double.
   JsonObject &addDouble(const std::string& key, double value) {
      char buffer[32];

      std::snprintf(buffer, sizeof(buffer), "%.17g", value);
      appendKey(key);
      body_ += buffer;
      return *this;
   }

   JsonObject &addBool(const std::string& key, bool value) {
      appendKey(key);
      body_ += value ? "true" : "false";
      return *this;
   }

   // Готовый фрагмент JSON (вложенный объект либо массив)
   JsonObject &addRaw(const std::string& key, const std::string& rawJson) {
      appendKey(key);
      body_ += rawJson;
      return *this;
   }

   bool empty() const noexcept {
      return body_.empty();
   }

   std::string str() const {
      return "{" + body_ + "}";
   }

private:

   void appendKey(const std::string& key) {
      if (!body_.empty()) {
         body_ += ", ";
      }
      body_ += '"';
      body_ += jsonEscape(key);
      body_ += "\": ";
   }

   std::string body_;
};
} // namespace glonass_service

#endif // SERVICE_JSON_WRITER_H
