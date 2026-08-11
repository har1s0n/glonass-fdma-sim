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

// --- Параметры тракта L1OC (реестр §0.1; ИКД ГЛОНАСС L1OC ред. 1.0) ---
constexpr std::int64_t chipRateL1OC = 1023000;         // f_T1, чип/с (поз.27; [ИКД-L1OC] 2.1.2)
constexpr int codeLengthD           = 1023;            // N_d (поз.29; [ИКД-L1OC] 2.2.1)
constexpr int codeLengthP           = 4092;            // N_p (поз.30; [ИКД-L1OC] 2.2.2)
constexpr int multiplexPeriod       = 2 * codeLengthP; // M = 8184 чипов уплотнения (А_L1OC.2)
constexpr int satelliteCount        = 64;              // j = 0…63, j = 0 резервный (поз.28)

// --- Навигационное сообщение L1OC (блок Б_L1OC) ---
constexpr std::int64_t symbolRateL1OC = 250; // R_с, симв/с; символ СК 4 мс (поз.37, 38; [ИКД-L1OC] 2.1.2, 2.3)
constexpr int maxPayloadBitsL1OC      = 339; // максимум бит ЦИ — строка 2-го типа (поз.37; [ИКД-L1OC] табл. 4.2, 4.3, 4.5)
constexpr int maxLineSymbolsL1OC      = 750; // максимум L_с = 2·n_с — строка 2-го типа (Б_L1OC.3)
} // namespace glonass
