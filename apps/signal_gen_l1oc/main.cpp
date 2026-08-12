// apps/signal_gen_l1oc/main.cpp — модуль запуска генерации сигнала тракта L1OC
#include "glonass/iq_sink.h"
#include "glonass/signal_source_l1oc.h"
#include "glonass/source_config_l1oc.h"
#include "glonass/types.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
glonass::PayloadProviderL1OC zeroPayloadProviderL1OC() {
   return [](std::int64_t /*lineIndex*/) {
             return glonass::LineContentL1OC{}; // lineType = normal, ЦИ = 0 (value-init)
   };
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

// --j : "a:b" (диапазон) либо "a,b,c" (список). j ∈ {1,…,63}: j = 0 резервный (поз.28).
std::vector<int> parseSatellites(const std::string& s) {
   std::vector<int> j;

   if (s.find(':') != std::string::npos) {
      const auto parts = split(s, ':');

      if (parts.size() != 2) {
         throw std::runtime_error("--j диапазон: ожидается a:b");
      }
      const int a = std::stoi(parts[0]);
      const int b = std::stoi(parts[1]);

      if (a > b) {
         throw std::runtime_error("--j: a > b");
      }

      for (int v = a; v <= b; ++v) {
         j.push_back(v);
      }
   } else {
      for (const auto& p : split(s, ',')) {
         j.push_back(std::stoi(p));
      }
   }

   if (j.empty()) {
      throw std::runtime_error("--j: пустой набор");
   }

   for (int v : j) {
      if ((v < 1) || (v >= glonass::satelliteCount)) {
         throw std::runtime_error("--j: системный номер вне {1,…,63} (j = 0 резервный): "
                                  + std::to_string(v));
      }
   }
   std::vector<int> sorted = j; // повтор j недопустим: J — множество (Д_L1OC.2)
   std::sort(sorted.begin(), sorted.end());

   if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end()) {
      throw std::runtime_error("--j: повторяющийся системный номер");
   }
   return j;
}

// --amp / --phi0 : скаляр (на все НКА) либо список по |J|.
std::vector<double> parsePerSatellite(const std::string& s, std::size_t count, const char* key) {
   std::vector<double> v;

   for (const auto& p : split(s, ',')) {
      v.push_back(std::stod(p));
   }

   if (v.size() == 1) {
      return std::vector<double> (count, v[0]);
   }

   if (v.size() != count) {
      throw std::runtime_error(std::string(key) + ": число значений != |J|");
   }
   return v;
}
} // namespace

int main(int argc, char** argv) try {
   std::int64_t fs       = 20000000;                 // Fs = 20,0 МГц (§ 0.1 поз.35)
   std::int64_t f0       = glonass::carrierFreqL1OC; // f₀ = f_L1OC ⇒ Δf_j = 0 (поз.26)
   std::int64_t n0       = 0;
   std::string  js       = "1:24";                   // J = {1,…,24} — сценарий Д_L1OC.10
   std::string  amps     = "1";                      // A_j = 1 (поз.24)
   std::string  phases   = "0";                      // φ_{0,j} = 0 (поз.46: π/2 — квадратура)
   long long    nSamples = 160000;                   // окно 8 мс — период замыкания кодовой фазы
   bool   nGiven         = false;
   double seconds        = 0.0;
   bool   secondsGiven   = false;
   std::string outPath   = "glonass_l1oc.cf32";
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
         f0 = std::stoll(next("--f0"));
      } else if (key == "--n0") {
         n0 = std::stoll(next("--n0"));
      } else if (key == "--j") {
         js = next("--j");
      } else if (key == "--amp") {
         amps = next("--amp");
      } else if (key == "--phi0") {
         phases = next("--phi0");
      } else if (key == "--n") {
         nSamples = std::stoll(next("--n")); nGiven = true;
      } else if (key == "--seconds") {
         seconds = std::stod(next("--seconds")); secondsGiven = true;
      } else if (key == "--out") {
         outPath = next("--out");
      } else if (key == "--sidecar") {
         sidePath = next("--sidecar");
      } else if ((key == "--help") ||
                 (key == "-h")) {
         std::cout << "glonass_signal_gen_l1oc [--fs Hz] [--f0 Hz] [--n0 N] [--j a:b|list]\n"
                      "                        [--amp v|list] [--phi0 rad|list]\n"
                      "                        [--n N | --seconds S]\n"
                      "                        [--out file.cf32] [--sidecar file.json]\n";
         return 0;
      } else { throw std::runtime_error("неизвестный аргумент: " + key); }
   }

   if (fs <= 0) {
      throw std::runtime_error("--fs должно быть > 0");
   }

   // Условие представимости (В.2, поз.34): |Δf_j| + B_model ≤ Fs/2; при f₀ = f_L1OC ⇒ Fs ≥ 4,092 МГц.
   const std::int64_t residualFreq = glonass::carrierFreqL1OC - f0;

   if (std::llabs(residualFreq) + glonass::modelBandwidthL1OC > fs / 2) {
      throw std::runtime_error("нарушено условие представимости В.2: |Δf| + B_model > Fs/2");
   }

   if (fs < glonass::symbolRateL1OC) {
      throw std::runtime_error("нарушено предусловие Б_L1OC.8: Fs < R_с");
   }

   if (n0 < 0) {
      throw std::runtime_error("--n0 должно быть ≥ 0");
   }
   const std::vector<int> J     = parseSatellites(js);
   const std::vector<double> A  = parsePerSatellite(amps,   J.size(), "--amp");
   const std::vector<double> Ph = parsePerSatellite(phases, J.size(), "--phi0");

   // Длительность: --n (приоритет) либо --seconds → отсчёты.
   if (secondsGiven && !nGiven) {
      nSamples = static_cast<long long> (std::llround(seconds * static_cast<double> (fs)));
   }

   if (nSamples <= 0) {
      throw std::runtime_error("число отсчётов должно быть > 0");
   }

   if (sidePath.empty()) {
      sidePath = outPath + ".json";
   }

   // Сборка SourceConfigL1OC; порядок по возрастанию j (Д_L1OC.8) — источник сортирует сам,
   // здесь порядок фиксируется для sidecar (соответствие satellites ↔ amplitudes).
   std::vector<std::size_t> order(J.size());

   for (std::size_t i = 0; i < J.size(); ++i) {
      order[i] = i;
   }
   std::sort(order.begin(), order.end(), [&J](std::size_t a, std::size_t b) {
      return J[a] < J[b];
   });

   glonass::SourceConfigL1OC cfg;
   cfg.sampleRate        = fs;
   cfg.referenceFreq     = f0;
   cfg.globalStartSample = static_cast<glonass::SampleIndex> (n0);

   std::vector<int> orderedSatellites;
   std::vector<double> orderedAmplitudes;

   for (std::size_t idx : order) {
      glonass::SatelliteConfigL1OC sc;
      sc.satellite         = J[idx];
      sc.amplitude         = A[idx];
      sc.codePhaseInit     = 0.0; // φ_{c0,j} = 0 (поз.47)
      sc.initialPhase      = Ph[idx];
      sc.payloadOfLineL1OC = zeroPayloadProviderL1OC();
      cfg.satellites.push_back(std::move(sc));
      orderedSatellites.push_back(J[idx]);
      orderedAmplitudes.push_back(A[idx]);
   }

   // Предусловия ядра: |J|≥1, ΣA²>0, A_j≥0, Fs>0, n0≥0, j∈{1,…,63} без повторов, В.9.
   glonass::SignalSourceL1OC source(cfg);

   // Прогон N отсчётов → потоковая запись CF32 (вся выборка в ОЗУ не держится).
   glonass::IqSink sink(outPath);

   for (long long r = 0; r < nSamples; ++r) {
      sink.writeSample(source.step()); // Фаза1(съём ВСЕХ НКА на n → Д_L1OC)→(I,Q); Фаза2(→ n+1)
   }
   sink.close();

   // Sidecar по фактическому прогону (η — из ядра, не пересчитывается здесь).
   glonass::IqMetadata meta;
   meta.sampleRateHz = fs;
   meta.rfCenterHz   = f0;
   meta.band         = "L1OC";
   meta.n0           = n0;
   meta.numSamples   = sink.samplesWritten();
   meta.satellites   = orderedSatellites;
   meta.amplitudes   = orderedAmplitudes;
   meta.eta          = source.normalizationFactor();
   meta.payload      = "zero";
   glonass::writeSidecarJson(sidePath, meta);

   std::cout << "Записано отсчётов: " << sink.samplesWritten()
             << " (" << sink.samplesWritten() * 8 << " байт CF32)\n"
             << "Бинарник: " << outPath << "\n"
             << "Sidecar:  " << sidePath << "\n"
             << "Fs=" << fs << " Гц, f0=" << f0 << " Гц, тракт=L1OC"
             << ", |J|=" << J.size() << ", n0=" << n0 << ", eta=" << meta.eta << "\n";
   return 0;
}
catch (const std::exception& e) {
   std::cerr << "Ошибка: " << e.what() << "\n";
   return 1;
}
