#ifndef REQUEST_PARAMS_L1OC_H
#define REQUEST_PARAMS_L1OC_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "glonass/types.h"

namespace glonass_params {
inline constexpr std::int64_t defaultSampleRate    = 20000000;                 // Fs = 20,0 МГц (§ 0.1 поз.35)
inline constexpr std::int64_t defaultReferenceFreq = glonass::carrierFreqL1OC; // f₀ = f_L1OC ⇒ Δf_j = 0 (поз.26)
inline constexpr const char*  defaultSatellites    = "1:24";                   // J = {1,…,24} — сценарий Д_L1OC.10
inline constexpr const char*  defaultAmplitudes    = "1";                      // A_j = 1 (поз.24)
inline constexpr const char*  defaultPhases        = "0";                      // φ_{0,j} = 0 (поз.46: π/2 — квадратура)

// Разряд отказа. Определяет код состояния HTTP
enum class RejectKind {
   badValue,    // значение отсутствует, не разбирается либо вне допустимого диапазона (400)
   unrealizable // значения формально корректны, конфигурация нереализуема (422)
};

class ParamError : public std::runtime_error {
public:

   ParamError(RejectKind kind, std::string field, const std::string& message)
      : std::runtime_error(message), kind_(kind), field_(std::move(field)) {}

   RejectKind kind() const noexcept {
      return kind_;
   }

   const std::string &field() const noexcept {
      return field_;
   }

private:

   RejectKind kind_;
   std::string field_;
};

namespace detail {
inline std::vector<std::string> split(const std::string& text, char separator) {
   std::vector<std::string> parts;
   std::string item;
   std::istringstream stream(text);

   while (std::getline(stream, item, separator)) {
      parts.push_back(item);
   }
   return parts;
}

inline std::int64_t parseIntStrict(const std::string& text, const char* key) {
   std::size_t consumed = 0;
   long long   value    = 0;

   try {
      value = std::stoll(text, &consumed);
   } catch (const std::exception&) {
      throw ParamError(RejectKind::badValue, key,
                       std::string(key) + ": значение не является целым числом: " + text);
   }
   if ((consumed != text.size()) || text.empty()) {
      throw ParamError(RejectKind::badValue, key,
                       std::string(key) + ": значение не является целым числом: " + text);
   }
   return value;
}

inline double parseDoubleStrict(const std::string& text, const char* key) {
   std::size_t consumed = 0;
   double value         = 0.0;

   try {
      value = std::stod(text, &consumed);
   } catch (const std::exception&) {
      throw ParamError(RejectKind::badValue, key,
                       std::string(key) + ": значение не является числом: " + text);
   }
   if ((consumed != text.size()) || text.empty()) {
      throw ParamError(RejectKind::badValue, key,
                       std::string(key) + ": значение не является числом: " + text);
   }
   return value;
}
} // namespace detail

// Целое значение параметра (--fs, --f0, --n0)
inline std::int64_t parseInteger(const std::string& text, const char* key) {
   return detail::parseIntStrict(text, key);
}

// Вещественное значение параметра (--t)
inline double parseReal(const std::string& text, const char* key) {
   return detail::parseDoubleStrict(text, key);
}

// j: «a:b» (диапазон) либо «a,b,c» (список). j ∈ {1,…,63}: j = 0 резервный (§ 0.1 поз.28).
// Повтор j недопустим: J — множество (Д_L1OC.2). Порядок следования сохраняется.
inline std::vector<int> parseSatellites(const std::string& text, const char* key) {
   std::vector<int> satellites;

   if (text.find(':') != std::string::npos) {
      const auto parts = detail::split(text, ':');

      if (parts.size() != 2) {
         throw ParamError(RejectKind::badValue, key, std::string(key) + " диапазон: ожидается a:b");
      }
      const std::int64_t first = detail::parseIntStrict(parts[0], key);
      const std::int64_t last  = detail::parseIntStrict(parts[1], key);

      if (first > last) {
         throw ParamError(RejectKind::badValue, key, std::string(key) + ": a > b");
      }

      for (std::int64_t value = first; value <= last; ++value) {
         satellites.push_back(static_cast<int> (value));
      }
   } else {
      for (const auto& part : detail::split(text, ',')) {
         satellites.push_back(static_cast<int> (detail::parseIntStrict(part, key)));
      }
   }

   if (satellites.empty()) {
      throw ParamError(RejectKind::badValue, key, std::string(key) + ": пустой набор");
   }

   for (const int value : satellites) {
      if ((value < 1) || (value >= glonass::satelliteCount)) {
         throw ParamError(RejectKind::badValue, key,
                          std::string(key) + ": системный номер вне {1,…,63} (j = 0 резервный): "
                          + std::to_string(value));
      }
   }
   std::vector<int> sorted = satellites;
   std::sort(sorted.begin(), sorted.end());

   if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end()) {
      throw ParamError(RejectKind::badValue, key, std::string(key) + ": повторяющийся системный номер");
   }
   return satellites;
}

// amp / phi0: скаляр (на все НКА) либо список по |J|
inline std::vector<double> parsePerSatellite(const std::string& text, std::size_t count,
                                             const char* key) {
   std::vector<double> values;

   for (const auto& part : detail::split(text, ',')) {
      values.push_back(detail::parseDoubleStrict(part, key));
   }

   if (values.size() == 1) {
      return std::vector<double> (count, values[0]);
   }

   if (values.size() != count) {
      throw ParamError(RejectKind::badValue, key, std::string(key) + ": число значений != |J|");
   }
   return values;
}

inline void requireSampleRate(std::int64_t sampleRate, const char* key) {
   if (sampleRate <= 0) {
      throw ParamError(RejectKind::badValue, key, std::string(key) + " должно быть > 0");
   }
}

inline void requireStartSample(std::int64_t startSample, const char* key) {
   if (startSample < 0) {
      throw ParamError(RejectKind::badValue, key, std::string(key) + " должно быть ≥ 0");
   }
}

// A_j ≥ 0 (§ 0.1 поз.24) — диапазон значения; Σ A_j² > 0 — предусловие нормировки Д_L1OC.1:
// при нулевой сумме η не определена, конфигурация нереализуема.
inline void requireAmplitudes(const std::vector<double>& amplitudes, const char* key) {
   double sumSquares = 0.0;

   for (const double amplitude : amplitudes) {
      if (!(amplitude >= 0.0)) { // отрицание охватывает и NaN
         throw ParamError(RejectKind::badValue, key,
                          std::string(key) + ": относительная амплитуда должна быть ≥ 0");
      }
      sumSquares += amplitude * amplitude;
   }

   if (!(sumSquares > 0.0)) {
      throw ParamError(RejectKind::unrealizable, key,
                       "нарушено предусловие Д_L1OC.1: Σ A_j² = 0, коэффициент нормировки не определён");
   }
}

// Δf_j = f_L1OC − f₀ (В.4); при f₀ = f_L1OC ⇒ Δf_j = 0 (§ 0.1 поз.26; § 1 (1.7))
inline std::int64_t residualFreq(std::int64_t referenceFreq) noexcept {
   return glonass::carrierFreqL1OC - referenceFreq;
}

// Условие представимости В.2 (§ 0.1 поз.22, 34): |Δf_j| + B_model ≤ Fs/2; нестрогое — равенство
// принимается. Предусл.: Fs > 0. Форма «Fs/2» с целочисленным делением тождественна форме
// «2·(|Δf| + B_model) ≤ Fs» на целых операндах.
inline bool isRepresentable(std::int64_t sampleRate, std::int64_t referenceFreq) noexcept {
   const std::int64_t delta     = residualFreq(referenceFreq);
   const std::int64_t magnitude = (delta < 0) ? -delta : delta;

   return magnitude + glonass::modelBandwidthL1OC <= sampleRate / 2;
}

// Отказ по В.2 — для режимов, выполняющих прогон. Точка показателей режима А прогона не
// выполняет и вместо отказа выводит признак isRepresentable отдельным полем ответа.
inline void requireRepresentable(std::int64_t sampleRate, std::int64_t referenceFreq,
                                 const char* key) {
   if (!isRepresentable(sampleRate, referenceFreq)) {
      throw ParamError(RejectKind::unrealizable, key,
                       "нарушено условие представимости В.2: |Δf| + B_model > Fs/2");
   }
}

// Предусловие Б_L1OC.8: Fs ≥ R_с — не более одной границы символа СК на отсчёт
inline void requireSymbolRate(std::int64_t sampleRate, const char* key) {
   if (sampleRate < glonass::symbolRateL1OC) {
      throw ParamError(RejectKind::unrealizable, key,
                       "нарушено предусловие Б_L1OC.8: Fs < R_с");
   }
}
} // namespace glonass_params

#endif // REQUEST_PARAMS_L1OC_H
