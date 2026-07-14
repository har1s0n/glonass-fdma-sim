// apps/signal_gen/main.cpp — модуль запуска генерации сигналов.
// SourceConfig → SignalSource → прогон N отсчётов → потоковый экспорт CF32 + sidecar.
// Сценарий (полная сетка L1) — умолчания покрывают его целиком:
//   glonass_signal_gen --n 16368 --out glonass_l1.cf32
// Fs=16368000, band=L1, K=-7:6, A_k=1, n0=0, f0=auto(центр K), N=16368.

#include "glonass/iq_sink.h"
#include "glonass/signal_source.h"
#include "glonass/source_config.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
glonass::Band bandFromString(const std::string& s) {
   if (s == "L1") {
      return glonass::Band::L1OF;
   }

   if (s == "L2") {
      return glonass::Band::L2OF;
   }
   throw std::runtime_error("неизвестный --band (ожидается L1OF|L2OF): " + s);
}

glonass::PayloadProvider makeZeroPayloadProvider() {
   return [](auto&& /*lineIndex*/) {
             return std::array<std::uint8_t, 76>{}; // [ТП] тип поля payload; value-init ⇒ все нули
   };
}

// ─────────────────────────────────────────────────────────────────────────────────────────────

// Несущие ([ИКД] 3.3.1.1, табл. 3.1; подтверждено в Р2): f_k = nominal + k·step. Шаг чётный ⇒
// центр набора K целочислен. (Локальные константы — для авто-f0; при необходимости взять из ядра.)
struct BandPlan { std::int64_t nominalHz; std::int64_t stepHz; };
BandPlan bandPlan(const std::string& band) {
   if (band == "L1") {
      return { 1602000000LL, 562500LL }; // 1602 МГц, шаг 0,5625 МГц
   }

   if (band == "L2") {
      return { 1246000000LL, 437500LL }; // 1246 МГц, шаг 0,4375 МГц
   }
   throw std::runtime_error("неизвестный band: " + band);
}

std::vector<std::string> split(const std::string& s, char sep) {
   std::vector<std::string> out;
   std::string item;
   std::istringstream iss(s);

   while (std::getline(iss, item, sep)) {
      out.push_back(item);
   }
   return out;
}

// --k : "a:b" (диапазон) либо "a,b,c" (список).
std::vector<int> parseLiterals(const std::string& s) {
   std::vector<int> k;

   if (s.find(':') != std::string::npos) {
      const auto parts = split(s, ':');

      if (parts.size() != 2) {
         throw std::runtime_error("--k диапазон: ожидается a:b");
      }
      const int a = std::stoi(parts[0]);
      const int b = std::stoi(parts[1]);

      if (a > b) {
         throw std::runtime_error("--k: a > b");
      }

      for (int v = a; v <= b; ++v) {
         k.push_back(v);
      }
   } else {
      for (const auto& p : split(s, ',')) {
         k.push_back(std::stoi(p));
      }
   }

   if (k.empty()) {
      throw std::runtime_error("--k: пустой набор");
   }
   return k;
}

// --amp : скаляр (на все литеры) либо список по |K|.
std::vector<double> parseAmplitudes(const std::string& s, std::size_t kcount) {
   std::vector<double> a;

   for (const auto& p : split(s, ',')) {
      a.push_back(std::stod(p));
   }

   if (a.size() == 1) {
      return std::vector<double> (kcount, a[0]);
   }

   if (a.size() != kcount) {
      throw std::runtime_error("--amp: число значений != |K|");
   }
   return a;
}
} // namespace

int main(int argc, char** argv) try {
   std::int64_t fs       = 16368000;
   std::string  f0s      = "auto";
   std::string  band     = "L1";
   std::int64_t n0       = 0;
   std::string  ks       = "-7:6";
   std::string  amps     = "1";
   long long    nSamples = 16368; // один период кода (≈1 мс)
   bool   nGiven         = false;
   double seconds        = 0.0;
   bool   secondsGiven   = false;
   std::string outPath   = "glonass_l1.cf32";
   std::string sidePath;

   // Разбор --key value
   for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      auto next = [&](const char* k) -> std::string {
                     if (i + 1 >= argc) {
                        throw std::runtime_error(std::string("нет значения для ") + k);
                     }
                     return argv[++i];
                  };

      if      (key == "--fs") {
         fs = std::stoll(next("--fs"));
      } else if (key == "--f0") {
         f0s = next("--f0");
      } else if (key == "--band") {
         band = next("--band");
      } else if (key == "--n0") {
         n0 = std::stoll(next("--n0"));
      } else if (key ==
                 "--k") {
         ks = next("--k");
      } else if (key ==
                 "--amp") {
         amps = next("--amp");
      } else if (key ==
                 "--n") {
         nSamples = std::stoll(next("--n")); nGiven = true;
      } else if (key ==
                 "--seconds") {
         seconds = std::stod(next("--seconds")); secondsGiven = true;
      } else if (key ==
                 "--out") {
         outPath = next("--out");
      } else if (key ==
                 "--sidecar") {
         sidePath = next("--sidecar");
      } else if ((key == "--help") ||
                 (key == "-h")) {
         std::cout << "glonass_signal_gen [--fs Hz] [--f0 Hz|auto] [--band L1|L2] [--n0 N]\n"
                      "                   [--k a:b|list] [--amp v|list] [--n N | --seconds S]\n"
                      "                   [--out file.cf32] [--sidecar file.json]\n";
         return 0;
      } else { throw std::runtime_error("неизвестный аргумент: " + key); }
   }

   if (fs <= 0) {
      throw std::runtime_error("--fs должно быть > 0");
   }
   const std::vector<int> K    = parseLiterals(ks);
   const std::vector<double> A = parseAmplitudes(amps, K.size());

   // Длительность: --n (приоритет) либо --seconds → отсчёты.
   if (secondsGiven && !nGiven) {
      nSamples = static_cast<long long> (std::llround(seconds * static_cast<double> (fs)));
   }

   if (nSamples <= 0) {
      throw std::runtime_error("число отсчётов должно быть > 0");
   }

   // f0: auto = центр набора K (f_k = nominal + k·step; шаг чётный ⇒ центр целочислен).
   std::int64_t f0;

   if (f0s == "auto") {
      const BandPlan  bp = bandPlan(band);
      const long long lo = *std::min_element(K.begin(), K.end());
      const long long hi = *std::max_element(K.begin(), K.end());
      f0 = bp.nominalHz + (lo + hi) * bp.stepHz / 2; // (lo+hi)·step чётно ⇒ деление точное
   } else {
      f0 = std::stoll(f0s);
   }

   if (sidePath.empty()) {
      sidePath = outPath + ".json";
   }

   // Сборка SourceConfig (поля — по source_config.h).
   glonass::SourceConfig cfg;
   cfg.band              = bandFromString(band); // [ТП] Band
   cfg.sampleRate        = fs;
   cfg.referenceFreq     = f0;
   cfg.globalStartSample = static_cast<glonass::SampleIndex> (n0);

   for (std::size_t i = 0; i < K.size(); ++i) {
      glonass::LetterConfig lc;
      lc.letter        = K[i];
      lc.amplitude     = A[i];
      lc.codePhaseInit = 0.0;                       // v1
      lc.initialPhase  = 0.0;                       // v1
      lc.payloadOfLine = makeZeroPayloadProvider(); // [ТП] нулевой payload v1
      cfg.letters.push_back(std::move(lc));
   }

   // Предусловия ядра: |K|≥1, ΣA²>0, A_k≥0, Fs>0, n0≥0, на литеру В.9 (проверяются в конструкторе).
   glonass::SignalSource source(cfg);

   // Прогон N отсчётов → потоковая запись CF32 (вся выборка в ОЗУ не держится).
   glonass::IqSink sink(outPath);

   for (long long r = 0; r < nSamples; ++r) {
      sink.writeSample(source.step()); // Фаза1(съём ВСЕХ литер на n → Д)→(I,Q); Фаза2(→ n+1)
   }
   sink.close();

   // Sidecar по фактическому прогону (η — из ядра, не пересчитывается здесь).
   glonass::IqMetadata meta;
   meta.sampleRateHz = fs;
   meta.rfCenterHz   = f0;
   meta.band         = band;
   meta.n0           = n0;
   meta.numSamples   = sink.samplesWritten();
   meta.literals.assign(K.begin(), K.end());
   meta.amplitudes = A;
   meta.eta        = source.normalizationFactor();
   meta.payload    = "zero";
   glonass::writeSidecarJson(sidePath, meta);

   std::cout << "Записано отсчётов: " << sink.samplesWritten()
             << " (" << sink.samplesWritten() * 8 << " байт CF32)\n"
             << "Бинарник: " << outPath << "\n"
             << "Sidecar:  " << sidePath << "\n"
             << "Fs=" << fs << " Гц, f0=" << f0 << " Гц, band=" << band
             << ", |K|=" << K.size() << ", n0=" << n0 << ", eta=" << meta.eta << "\n";
   return 0;
}
catch (const std::exception& e) {
   std::cerr << "Ошибка: " << e.what() << "\n";
   return 1;
}
