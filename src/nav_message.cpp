#include <cassert>
#include <cstddef>
#include <utility>

#include "glonass/nav_message.h"

namespace glonass {
namespace {
// Списки проверочных сумм кода Хэмминга L1..L7 (Ч3 Б.4(1), [ИКД] Табл. 4.13), позиции ИКД 1..85.
constexpr int L1[] = { 9,  10,  12,  13,  15,  17,  19, 20, 22, 24, 26, 28, 30, 32, 34, 35, 37, 39, 41, 43, 45,
                       47, 49,  51,  53,  55,  57,  59, 61, 63, 65, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84 }; // 41
constexpr int L2[] = { 9,  11,  12,  14,  15,  18,  19, 21, 22, 25, 26, 29, 30, 33, 34, 36, 37, 40, 41, 44, 45,
                       48, 49,  52,  53,  56,  57,  60, 61, 64, 65, 67, 68, 71, 72, 75, 76, 79, 80, 83, 84 }; // 41
constexpr int L3[] = { 10, 11, 12, 16, 17, 18, 19, 23, 24, 25, 26, 31, 32, 33, 34, 38, 39, 40, 41,
                       46, 47, 48, 49, 54, 55, 56, 57, 62, 63, 64, 65, 69, 70, 71, 72, 77, 78, 79,80, 85 };   // 40
constexpr int L4[] = { 13, 14, 15, 16, 17, 18, 19, 27, 28, 29, 30, 31, 32, 33, 34,
                       42, 43, 44, 45, 46, 47, 48, 49, 58, 59, 60, 61, 62, 63, 64,65,
                       73, 74, 75, 76, 77, 78, 79, 80 };                                                      // 39
constexpr int L5[] = { 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34,
                       50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64,65,
                       81, 82, 83, 84, 85 };                                                                  // 36
constexpr int L6[] = { 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
                       51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65 };                          // 31
constexpr int L7[] = { 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85 };      // 20

// Таблица метки времени — компилируемая константа, вычисляется один раз (Ч3 Б.4(4), [ИКД] 3.3.2.2).
constexpr std::array<Bit, timeMarkLength> kTimeMark = detail::buildTimeMarkTable();
} // namespace

std::array<Bit, 8> hammingParity(const std::array<Bit, 85>& dPos) {
   // XOR символов позиций из списка (позиция p -> dPos[p-1]); диапазон p строго 9..85 (Б.4(1)).
   auto xorPositions = [&](const auto& positions) -> Bit {
                          Bit acc = 0;

                          for (int p : positions) {
                             acc = static_cast<Bit> (acc ^ dPos[static_cast<std::size_t> (p - 1)]);
                          }
                          return acc;
                       };

   std::array<Bit, 8> beta{};

   beta[0] = xorPositions(L1); // β1
   beta[1] = xorPositions(L2); // β2
   beta[2] = xorPositions(L3); // β3
   beta[3] = xorPositions(L4); // β4
   beta[4] = xorPositions(L5); // β5
   beta[5] = xorPositions(L6); // β6
   beta[6] = xorPositions(L7); // β7

   Bit b8 = 0;                 // β8 = β1⊕…⊕β7 ⊕ (⊕ d_pos[p], p=9..85)

   for (int r = 0; r < 7; ++r) {
      b8 = static_cast<Bit> (b8 ^ beta[r]);
   }

   for (int p = 9; p <= 85; ++p) {
      b8 = static_cast<Bit> (b8 ^ dPos[static_cast<std::size_t> (p - 1)]);
   }
   beta[7] = b8;
   return beta;
}

BuiltLine buildLine(const std::array<Bit, 76>& payload76, Bit relativeStateIn) {
   // (1) кодер Хэмминга (Ч3 Б.4(1)/Б.9): d_pos индексируется позицией ИКД (dPos[p-1])
   std::array<Bit, 85> dPos{};

   for (int p = 9; p <= 84; ++p) {
      dPos[static_cast<std::size_t> (p - 1)] = payload76[static_cast<std::size_t> (p - 9)]; // позиции 9..84
   }
   dPos[85 - 1] = 0;                                                                        // поз.85 — холостой «0» (Б.4(2))
   const std::array<Bit, 8> beta = hammingParity(dPos);

   for (int r = 1; r <= 8; ++r) {
      dPos[static_cast<std::size_t> (r - 1)] = beta[static_cast<std::size_t> (r - 1)]; // позиции 1..8 = β1..β8
   }

   // (2) порядок передачи s[t]=d_pos[85-t] + непрерывный относительный код (Ч3 Б.4(2)/Б.9)
   std::array<Bit, 85> rel{};
   Bit prev = relativeStateIn;

   for (int t = 0; t < 85; ++t) {
      const Bit s = dPos[static_cast<std::size_t> ((85 - t) - 1)]; // s[t] = d_pos[85-t]
      prev                              = static_cast<Bit> (prev ^ s);
      rel[static_cast<std::size_t> (t)] = prev;
   }
   const Bit relativeStateOut = rel[84]; // = relativeStateIn (инвариант чётности)

   // (3) меандр «начинается с 1» (Ч3 Б.4(3), [ИКД] рис. 3.6): b[2t]=r⊕1, b[2t+1]=r
   BuiltLine result{};

   for (int t = 0; t < 85; ++t) {
      const Bit r = rel[static_cast<std::size_t> (t)];
      result.lineSymbols[static_cast<std::size_t> (2 * t)]     = static_cast<Bit> (r ^ 1);
      result.lineSymbols[static_cast<std::size_t> (2 * t + 1)] = r;
   }

   // (4) метка времени ПСПМВ (Ч3 Б.4(4), [ИКД] 3.3.2.2) — генерация РСЛОС, 170..199
   for (int m = 0; m < timeMarkLength; ++m) {
      result.lineSymbols[static_cast<std::size_t> (170 + m)] = kTimeMark[static_cast<std::size_t> (m)];
   }

   result.relativeStateOut = relativeStateOut;
   return result;
}

void NavMessage::init(SampleIndex globalStartSample, std::int64_t sampleRate, PayloadProvider payloadOfLine) {
   assert(globalStartSample >= 0);    // n0 >= 0 (граница v1)
   assert(sampleRate >= messageRate); // предусл. Ч3: Fs >= R_m
   assert(payloadOfLine);             // провайдер содержания задан

   sampleRate_    = sampleRate;
   payloadOfLine_ = std::move(payloadOfLine);

   // Z0 = n0*R_m — умещается в int64 в рабочем диапазоне (mulMod НЕ требуется, в отличие от А.4(2))
   const std::int64_t Z0 = globalStartSample * messageRate;
   lineIndex_               = Z0 / (200 * sampleRate_);                      // ⌊Z0/(200*Fs)⌋
   messageSymbolIndex_      = static_cast<int> ((Z0 / sampleRate_) % 200);   // ⌊Z0/Fs⌋ mod 200
   messagePhaseAccumulator_ = static_cast<std::uint64_t> (Z0 % sampleRate_); // Z0 mod Fs

   const Bit relativeStateIn = 0;

   const BuiltLine built = buildLine(payloadOfLine_(lineIndex_), relativeStateIn);
   lineSymbols_      = built.lineSymbols;
   relativeStateOut_ = built.relativeStateOut;
}

Bit NavMessage::messageBit() const {
   return lineSymbols_[static_cast<std::size_t> (messageSymbolIndex_)]; // b_line[j] (Ч3 Б.5/Б.7)
}

void NavMessage::step() {
   // фаза сообщения (Ч3 Б.6/Б.8): R=P_m+R_m; ν=⌊R/Fs⌋; P_m<-R−ν*Fs; j<-j+ν. ν∈{0,1} при R_m<Fs.
   const std::uint64_t Fs = static_cast<std::uint64_t> (sampleRate_);
   const std::uint64_t R  = messagePhaseAccumulator_ + static_cast<std::uint64_t> (messageRate);
   const std::uint64_t nu = R / Fs;

   messagePhaseAccumulator_ = R - nu * Fs;
   const int jNext = messageSymbolIndex_ + static_cast<int> (nu);

   if (jNext <= 199) {
      messageSymbolIndex_ = jNext;
   } else {
      // событие конца строки j:199->0 (Ч3 Б.8): перенос состояния, инкремент, построение следующей строки
      const Bit relativeStateIn = relativeStateOut_; // перенос a_out -> a_in
      lineIndex_ += 1;
      const BuiltLine built = buildLine(payloadOfLine_(lineIndex_), relativeStateIn);
      lineSymbols_        = built.lineSymbols;
      relativeStateOut_   = built.relativeStateOut;
      messageSymbolIndex_ = 0;
   }
}

int NavMessage::messageSymbolIndex() const {
   return messageSymbolIndex_;
}

std::int64_t NavMessage::lineIndex()          const {
   return lineIndex_;
}

Bit NavMessage::relativeStateOut()   const {
   return relativeStateOut_;
}
} // namespace glonass
