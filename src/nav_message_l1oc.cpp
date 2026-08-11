#include <cassert>
#include <cstddef>
#include <utility>

#include "glonass/nav_message_l1oc.h"

namespace glonass {
namespace {
// СМВ — поле бит 1…12 строки (§ 0.1 поз.40; [ИКД-L1OC] 4.2.2.1)
constexpr int smvLength                           = 12;
constexpr std::array<Bit, smvLength> kTimeMarkSMV = { 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1 };

// Образующие полиномы ЦК — степени ненулевых коэффициентов (§ 0.1 поз.42; [ИКД-L1OC] 4.4–4.6):
// (250,234) и (125,109): g(X) = 1+X+X⁵+X⁶+X⁸+X⁹+X¹⁰+X¹¹+X¹³+X¹⁴+X¹⁶
// (375,351):             g(X) = 1+X+X³+X⁴+X⁵+X⁶+X⁷+X¹⁰+X¹¹+X¹⁴+X¹⁷+X¹⁸+X²³+X²⁴
constexpr int kG250[] = { 0, 1, 5, 6, 8, 9, 10, 11, 13, 14, 16 };
constexpr int kG375[] = { 0, 1, 3, 4, 5, 6, 7, 10, 11, 14, 17, 18, 23, 24 };

constexpr int maxInfoBits   = 351; // k = 12 + 339 — строка 2-го типа
constexpr int maxParityBits = 24;  // L = 24 — строка 2-го типа
constexpr int maxLineBits   = 375; // n_с = 351 + 24

// Маска сумматоров регистра ЦК: сумматор стоит после триггера t для каждого ненулевого g_t,
// t = 1…L−1 (Б_L1OC.4(1)); коэффициенты g_0 и g_L схемой не задаются. Бит (t−1) ↔ триггер t.
template<std::size_t N>
unsigned tapMaskOf(const int (& degrees)[N], int L) {
   unsigned mask = 0;

   for (std::size_t i = 0; i < N; ++i) {
      const int t = degrees[i];

      if ((t >= 1) && (t <= L - 1)) {
         mask |= 1u << (t - 1);
      }
   }
   return mask;
}

// Циклический код (Б_L1OC.1), (Б_L1OC.2); схемная форма [ИКД-L1OC] рисунки 4.4, 4.6, псевдокод
// Б_L1OC.9(2): регистр 1…L инициализируется нулями; на первых k сдвигах обратная связь
// fb = s[L] ⊕ вход замкнута; на последующих L сдвигах разомкнута и регистр выдаётся с s[L].
// Возвращает число проверочных бит L; parity[0…L−1]
int cyclicParity(const Bit* info, int infoLength, LineTypeL1OC lineType,
                 std::array<Bit, maxParityBits>& parity) {
   const bool isType2     = (lineType == LineTypeL1OC::anomalous2);
   const int  L           = isType2 ? 24 : 16;
   const unsigned tapMask = isType2 ? tapMaskOf(kG375, L) : tapMaskOf(kG250, L);

   std::array<Bit, maxParityBits + 1> s{}; // s[1…L]; элемент 0 не используется

   for (int idx = 0; idx < infoLength; ++idx) {
      const Bit fb = static_cast<Bit> (s[static_cast<std::size_t> (L)] ^ info[idx]);
      std::array<Bit, maxParityBits + 1> next{};
      next[1] = fb;

      for (int t = 1; t <= L - 1; ++t) {
         const Bit addend = (tapMask & (1u << (t - 1))) ? fb : static_cast<Bit> (0);
         next[static_cast<std::size_t> (t + 1)] =
            static_cast<Bit> (s[static_cast<std::size_t> (t)] ^ addend);
      }
      s = next;
   }

   for (int r = 0; r < L; ++r) { // обратная связь разомкнута
      parity[static_cast<std::size_t> (r)] = s[static_cast<std::size_t> (L)];

      for (int t = L; t > 1; --t) {
         s[static_cast<std::size_t> (t)] = s[static_cast<std::size_t> (t - 1)];
      }
      s[1] = 0;
   }
   return L;
}

// Свёрточный кодер (133,171), кодовое ограничение 7 (Б_L1OC.3)–(Б_L1OC.6);
void convEncodeLine(const Bit* line, int lineBitCount, ConvStateL1OC& state,
                    std::array<Bit, maxLineSymbolsL1OC>& lineSymbols) {
   for (int t = 0; t < lineBitCount; ++t) {
      const Bit u = line[t];

      // v₁[t] = u ⊕ S[2] ⊕ S[3] ⊕ S[5] ⊕ S[6] (Б_L1OC.3); b_line[2t] = v₁[t] (Б_L1OC.6)
      lineSymbols[static_cast<std::size_t> (2 * t)] =
         static_cast<Bit> (u ^ state[1] ^ state[2] ^ state[4] ^ state[5]);
      // v₂[t] = u ⊕ S[1] ⊕ S[2] ⊕ S[3] ⊕ S[6] (Б_L1OC.4); b_line[2t+1] = v₂[t]
      lineSymbols[static_cast<std::size_t> (2 * t + 1)] =
         static_cast<Bit> (u ^ state[0] ^ state[1] ^ state[2] ^ state[5]);

      for (int tau = 5; tau > 0; --tau) { // S[τ] ← S[τ−1], τ = 6…2 (Б_L1OC.5)
         state[static_cast<std::size_t> (tau)] = state[static_cast<std::size_t> (tau - 1)];
      }
      state[0] = u;                       // S[1] ← u[t]
   }
}
} // namespace

int lineInfoBits(LineTypeL1OC lineType) {
   switch (lineType) {
     case LineTypeL1OC::normal:     return 222; // нормальная строка, 2 с
     case LineTypeL1OC::anomalous1: return 97;  // строка 1-го типа, 1 с
     case LineTypeL1OC::anomalous2: return 339; // строка 2-го типа, 3 с
   }
   assert(false);                               // тип строки вне перечисления (поз.37)
   return 0;
}

int lineBits(LineTypeL1OC lineType) {
   switch (lineType) {
     case LineTypeL1OC::normal:     return 250; // (250,234)
     case LineTypeL1OC::anomalous1: return 125; // (125,109)
     case LineTypeL1OC::anomalous2: return 375; // (375,351)
   }
   assert(false);
   return 0;
}

BuiltLineL1OC buildLineL1OC(const LineContentL1OC& lineContent, const ConvStateL1OC& convStateIn) {
   const int infoBits     = lineInfoBits(lineContent.lineType);
   const int lineBitCount = lineBits(lineContent.lineType);

   // (1) информационный блок: СМВ (12 бит) + ЦИ; k = 234 / 109 / 351 (Б_L1OC.4(1))
   std::array<Bit, maxLineBits> line{};

   for (int t = 0; t < smvLength; ++t) {
      line[static_cast<std::size_t> (t)] = kTimeMarkSMV[static_cast<std::size_t> (t)];
   }

   for (int t = 0; t < infoBits; ++t) {
      line[static_cast<std::size_t> (smvLength + t)] =
         lineContent.payloadCI[static_cast<std::size_t> (t)];
   }
   const int infoLength = smvLength + infoBits;

   assert(infoLength <= maxInfoBits);

   // (2) циклический код: d[1…n_с] = m[1…k] ‖ r_{L−1}, …, r_0 (Б_L1OC.2)
   std::array<Bit, maxParityBits> parity{};
   const int parityLength = cyclicParity(line.data(), infoLength, lineContent.lineType, parity);

   assert(infoLength + parityLength == lineBitCount);

   for (int r = 0; r < parityLength; ++r) {
      line[static_cast<std::size_t> (infoLength + r)] = parity[static_cast<std::size_t> (r)];
   }

   // (3) свёрточный кодер; состояние на входе строки — после последнего бита предыдущей (Б_L1OC.4(2))
   BuiltLineL1OC result{};

   result.convStateOut = convStateIn;
   convEncodeLine(line.data(), lineBitCount, result.convStateOut, result.lineSymbols);
   result.lineLength = 2 * lineBitCount; // L_с = 2·n_с
   return result;
}

void NavMessageL1OC::initMessageAtSampleL1OC(SampleIndex         globalStartSample,
                                             std::int64_t        sampleRate,
                                             PayloadProviderL1OC payloadOfLineL1OC) {
   assert(globalStartSample >= 0);       // n0 >= 0
   assert(sampleRate >= symbolRateL1OC); // предусл. Ч3 Б_L1OC.8: Fs >= R_с
   assert(payloadOfLineL1OC);            // слой содержания задан (Б_L1OC.11)

   sampleRate_        = sampleRate;
   payloadOfLineL1OC_ = std::move(payloadOfLineL1OC);

   // Z0 = n0*R_с — умещается в int64 в рабочем диапазоне (mulMod не требуется)
   const std::int64_t Z0 = globalStartSample * symbolRateL1OC;

   // регулярный поток 2-секундных строк (Б_L1OC.4(4)): L_{с,j} = 500
   lineLength_             = 2 * lineBits(LineTypeL1OC::normal);
   lineIndex_              = Z0 / (static_cast<std::int64_t> (lineLength_) * sampleRate_);
   convSymbolIndex_        = static_cast<int> ((Z0 / sampleRate_) % lineLength_);
   symbolPhaseAccumulator_ = static_cast<std::uint64_t> (Z0 % sampleRate_);
   convEncoderState_       = ConvStateL1OC{}; // S_j[1..6] = 0 при запуске модели (поз.41)

   const LineContentL1OC content = payloadOfLineL1OC_(lineIndex_);

   // координаты запуска выведены для регулярного потока; аномальная первая строка их нарушает
   assert(content.lineType == LineTypeL1OC::normal);

   const BuiltLineL1OC built = buildLineL1OC(content, convEncoderState_);
   lineSymbols_  = built.lineSymbols;
   convStateOut_ = built.convStateOut;
}

Bit NavMessageL1OC::convSymbol() const {
   return lineSymbols_[static_cast<std::size_t> (convSymbolIndex_)]; // b_j[n] (Б_L1OC.8)
}

Bit NavMessageL1OC::overlaySymbol() const {
   assert(sampleRate_ > 0);
   // o_j[n] = ⌊2·P_{s,j}[n]/Fs⌋: 0 на первой половине символа СК, 1 — на второй (Б_L1OC.9);
   // собственного состояния оверлейный код не имеет (Б_L1OC.3)
   return static_cast<Bit> ((2 * symbolPhaseAccumulator_) / static_cast<std::uint64_t> (sampleRate_));
}

void NavMessageL1OC::step() {
   // фаза символа СК (Б_L1OC.10): R = P_s+R_с; ν = ⌊R/Fs⌋; P_s <- R−ν·Fs; w <- w+ν. ν∈{0,1} при Fs>=R_с
   const std::uint64_t Fs = static_cast<std::uint64_t> (sampleRate_);
   const std::uint64_t R  = symbolPhaseAccumulator_ + static_cast<std::uint64_t> (symbolRateL1OC);
   const std::uint64_t nu = R / Fs;

   symbolPhaseAccumulator_ = R - nu * Fs;
   const int wNext = convSymbolIndex_ + static_cast<int> (nu);

   if (wNext <= lineLength_ - 1) {
      convSymbolIndex_ = wNext;
   } else {
      // событие конца строки w: L_{с,j}−1 → 0 (Б_L1OC.8): перенос состояния регистра СК,
      // инкремент номера строки, построение следующей строки средствами слоя содержания
      convEncoderState_ = convStateOut_;
      lineIndex_       += 1;
      const BuiltLineL1OC built = buildLineL1OC(payloadOfLineL1OC_(lineIndex_), convEncoderState_);
      lineSymbols_     = built.lineSymbols;
      convStateOut_    = built.convStateOut;
      lineLength_      = built.lineLength; // L_{с,j} = 2·n_с(тип следующей строки)
      convSymbolIndex_ = 0;
   }
}

int NavMessageL1OC::convSymbolIndex() const {
   return convSymbolIndex_;
}

std::int64_t NavMessageL1OC::lineIndex() const {
   return lineIndex_;
}

const ConvStateL1OC &NavMessageL1OC::convStateOut() const {
   return convStateOut_;
}

int NavMessageL1OC::lineLength() const {
   return lineLength_;
}

std::uint64_t NavMessageL1OC::symbolPhaseAccumulator() const {
   return symbolPhaseAccumulator_;
}
} // namespace glonass
