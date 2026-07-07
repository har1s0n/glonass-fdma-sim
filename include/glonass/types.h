#pragma once
#include <complex>
#include <cstdint>

namespace glonass {
using SampleIndex  = std::int64_t;           // n, n0, r (§0.2)
using Bit          = std::uint8_t;           // codeBit/messageBit/modulationBit in {0,1}
using OutputSample = std::complex<float>;    // u[n] на выходе Д -> float32-эталон (Д.9)
// внутренние комплексные (carrier, letterSample, transmitterSample) - std::complex<double>

constexpr int codeLength           = 511;    // N
constexpr std::int64_t codeRate    = 511000; // R_c, симв/с
constexpr std::int64_t messageRate = 100;    // R_m, симв/с
constexpr int phaseBits            = 32;     // B
constexpr int timeMarkLength       = 30;     // длина ПСПМВ (укороч. M-послед. 31->30), [ИКД] 3.3.2.2
} // namespace glonass
