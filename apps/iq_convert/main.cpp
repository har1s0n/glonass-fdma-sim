// apps/iq_convert/main.cpp — конвертер записи I/Q: CF32 LE → CS16 LE.
// Назначение: подача выхода модели во внешний программный приёмник, входные форматы которого
// ограничены целочисленными. Вызывается отдельной командой
#include "convert.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <ios>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr std::size_t blockSamples = 1U << 16; // отсчётов в блоке; обработка потоковая
constexpr std::size_t bytesPerCf32 = 8;        // float32 I + float32 Q
constexpr std::size_t bytesPerCs16 = 4;        // int16 I + int16 Q

std::ifstream openInput(const std::string& path, std::uintmax_t& sizeBytes) {
   std::ifstream in(path, std::ios::binary | std::ios::in);

   if (!in) {
      throw std::runtime_error("не удалось открыть для чтения: " + path);
   }
   in.seekg(0, std::ios::end);
   sizeBytes = static_cast<std::uintmax_t> (in.tellg());
   in.seekg(0, std::ios::beg);
   return in;
}

// Проход 1: максимум |x| по обеим координатам всей записи. Выполняется только при отсутствии
// ключа --scale: масштаб под полную шкалу определяется по фактическому пику записи.
double peakAbsOfFile(const std::string& path, std::size_t sampleCount) {
   std::uintmax_t sizeBytes = 0;
   std::ifstream  in        = openInput(path, sizeBytes);
   std::vector<unsigned char> buffer(blockSamples * bytesPerCf32);
   double peak = 0.0;

   for (std::size_t done = 0; done < sampleCount;) {
      const std::size_t chunk = std::min(blockSamples, sampleCount - done);

      in.read(reinterpret_cast<char*> (buffer.data()),
              static_cast<std::streamsize> (chunk * bytesPerCf32));

      if (!in) {
         throw std::runtime_error("ошибка чтения: " + path);
      }
      const double blockPeak = glonass_tools::peakAbsOfBlock(buffer.data(), chunk);

      if (blockPeak > peak) {
         peak = blockPeak;
      }
      done += chunk;
   }
   return peak;
}

// Проход 2: квантование и запись.
void convertFile(const std::string& inPath, const std::string& outPath,
                 std::size_t sampleCount, double scale) {
   std::uintmax_t sizeBytes = 0;
   std::ifstream  in        = openInput(inPath, sizeBytes);
   std::ofstream  out(outPath, std::ios::binary | std::ios::out | std::ios::trunc);

   if (!out) {
      throw std::runtime_error("не удалось открыть для записи: " + outPath);
   }
   std::vector<unsigned char> inBuffer(blockSamples * bytesPerCf32);
   std::vector<unsigned char> outBuffer(blockSamples * bytesPerCs16);

   for (std::size_t done = 0; done < sampleCount;) {
      const std::size_t chunk = std::min(blockSamples, sampleCount - done);

      in.read(reinterpret_cast<char*> (inBuffer.data()),
              static_cast<std::streamsize> (chunk * bytesPerCf32));

      if (!in) {
         throw std::runtime_error("ошибка чтения: " + inPath);
      }

      glonass_tools::convertBlockCs16(inBuffer.data(), chunk, scale, outBuffer.data());
      out.write(reinterpret_cast<const char*> (outBuffer.data()),
                static_cast<std::streamsize> (chunk * bytesPerCs16));

      if (!out) {
         throw std::runtime_error("ошибка записи: " + outPath);
      }
      done += chunk;
   }
   out.flush();

   if (!out) {
      throw std::runtime_error("ошибка закрытия: " + outPath);
   }
}
} // namespace

int main(int argc, char** argv) try {
   std::string inPath;
   std::string outPath;
   double scale      = 0.0;
   bool   scaleGiven = false;

   for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      auto next = [&](const char* k) -> std::string {
                     if (i + 1 >= argc) {
                        throw std::runtime_error(std::string("нет значения для ") + k);
                     }
                     return argv[++i];
                  };

      if (key == "--in") {
         inPath = next("--in");
      } else if (key == "--out") {
         outPath = next("--out");
      } else if (key == "--scale") {
         scale = std::stod(next("--scale")); scaleGiven = true;
      } else if ((key == "--help") ||
                 (key == "-h")) {
         std::cout << "glonass_iq_convert --in file.cf32 --out file.cs16 [--scale k]\n"
                      "  CF32 LE (I,Q float32) -> CS16 LE (I,Q int16), знак Q не инвертируется.\n"
                      "  Без --scale масштаб определяется по пику записи: k = 32767/max|x|.\n";
         return 0;
      } else { throw std::runtime_error("неизвестный аргумент: " + key); }
   }

   if (inPath.empty() || outPath.empty()) {
      throw std::runtime_error("требуются --in и --out (см. --help)");
   }

   if (scaleGiven && (scale <= 0.0)) {
      throw std::runtime_error("--scale должен быть > 0");
   }
   std::uintmax_t sizeBytes = 0;
   { std::ifstream probe = openInput(inPath, sizeBytes); }

   if (sizeBytes == 0) {
      throw std::runtime_error("пустой входной файл: " + inPath);
   }

   if (sizeBytes % bytesPerCf32 != 0) {
      throw std::runtime_error("размер входного файла не кратен 8 байтам CF32");
   }
   const std::size_t sampleCount = static_cast<std::size_t> (sizeBytes / bytesPerCf32);

   double peak = 0.0;

   if (!scaleGiven) {
      peak  = peakAbsOfFile(inPath, sampleCount);
      scale = glonass_tools::scaleForPeakCs16(peak);
   }
   convertFile(inPath, outPath, sampleCount, scale);

   std::cout << "Отсчётов: " << sampleCount
             << " (" << sizeBytes << " байт CF32 -> " << sampleCount * bytesPerCs16
             << " байт CS16)\n"
             << "Вход:  " << inPath  << "\n"
             << "Выход: " << outPath << "\n";

   if (scaleGiven) {
      std::cout << "Масштаб: " << scale << " (задан ключом --scale)\n";
   } else {
      std::cout << "Пик записи: " << peak << ", масштаб: " << scale
                << " (полная шкала CS16)\n";
   }
   return 0;
}
catch (const std::exception& e) {
   std::cerr << "Ошибка: " << e.what() << "\n";
   return 1;
}
