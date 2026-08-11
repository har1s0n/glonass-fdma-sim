#include "glonass/ranging_code_l1oc.h"
#include "glonass/numeric.h"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <string_view>

namespace glonass {
namespace {
constexpr int maxRegisterLength = 12; // L1 = 12 у ДК_L1OCp (поз.30)

// Регистр сдвига в форме Фибоначчи (раскрыто в А_L1OC.4(1)):
// выход — последний триггер s[L], съём ДО сдвига; обратная связь считается по состоянию
// до сдвига; начальное состояние записывается слева направо на триггеры 1…L.
// Индексы массива 0…L-1 соответствуют триггерам 1…L.
class FibonacciRegister {
public:

   FibonacciRegister(int length, std::initializer_list<int> taps, std::string_view initialState)
      : length_(length), tapMask_(maskOfTaps(length, taps)) {
      assert(static_cast<int> (initialState.size()) == length);

      for (int t = 1; t <= length; ++t) {
         s_[static_cast<std::size_t> (t - 1)] =
            static_cast<Bit> (initialState[static_cast<std::size_t> (t - 1)] - '0');
      }
   }

   // НС2 = двоичная запись номера j разрядности L, старший разряд — на триггер 1 (А_L1OC.4)
   FibonacciRegister(int length, std::initializer_list<int> taps, unsigned initialState)
      : length_(length), tapMask_(maskOfTaps(length, taps)) {
      for (int t = 1; t <= length; ++t) {
         s_[static_cast<std::size_t> (t - 1)] =
            static_cast<Bit> ((initialState >> (length - t)) & 1u);
      }
   }

   Bit output() const { // a[m] = s_L[m] (А_L1OC.1, А_L1OC.2)
      return s_[static_cast<std::size_t> (length_ - 1)];
   }

   void shift() {                          // (А_L1OC.3)
      Bit feedback = 0;

      for (int t = 1; t <= length_; ++t) { // ОС по состоянию ДО сдвига
         if (tapMask_ & (1u << (t - 1))) {
            feedback = static_cast<Bit> (feedback ^ s_[static_cast<std::size_t> (t - 1)]);
         }
      }

      for (int t = length_; t > 1; --t) { // s_t <- s_{t-1}, t = L…2
         s_[static_cast<std::size_t> (t - 1)] = s_[static_cast<std::size_t> (t - 2)];
      }
      s_[0] = feedback;                   // s_1 <- f
   }

private:

   static unsigned maskOfTaps(int length, std::initializer_list<int> taps) {
      assert(length > 0 && length <= maxRegisterLength);
      unsigned mask = 0;

      for (int t : taps) {
         assert(t >= 1 && t <= length);
         mask |= 1u << (t - 1);
      }
      return mask;
   }

   std::array<Bit, maxRegisterLength> s_{};
   int length_;
   unsigned tapMask_;
};

// ДК[j][m] = a[m] XOR b[m], m = 0…N-1 (А_L1OC.4); функция BuildCodeTable псевдокода А_L1OC.9.
// Регистры передаются по значению: после построения таблицы они не тактуются (А_L1OC.3).
template<std::size_t N>
void buildCodeTable(FibonacciRegister ca1, FibonacciRegister ca2, std::array<Bit, N>& table) {
   for (std::size_t m = 0; m < N; ++m) {
      table[m] = static_cast<Bit> (ca1.output() ^ ca2.output()); // съём ДО сдвига
      ca1.shift();
      ca2.shift();
   }
}
} // namespace

void RangingCodeL1OC::initCodeTablesL1OC(int j) {
   assert(j >= 0 && j < satelliteCount); // j = 0…63 (поз.28)

   // ДК_L1OCd: ЦА1 10 разрядов, отводы 7, 10, НС1 = 0011001000;
   //           ЦА2 10 разрядов, отводы 3, 7, 9, 10, НС2 = j ([ИКД-L1OC] 2.2.1, рисунок 2.4; поз.29)
   buildCodeTable(FibonacciRegister(10, { 7, 10 },       "0011001000"),
                  FibonacciRegister(10, { 3, 7, 9, 10 }, static_cast<unsigned> (j)),
                  codeTableD_);

   // ДК_L1OCp: ЦА1 12 разрядов, отводы 6, 8, 11, 12, НС1 = 000011000101;
   //           ЦА2 6 разрядов, отводы 1, 6, НС2 = j ([ИКД-L1OC] 2.2.2, рисунок 2.5; поз.30)
   buildCodeTable(FibonacciRegister(12, { 6, 8, 11, 12 }, "000011000101"),
                  FibonacciRegister(6, { 1, 6 }, static_cast<unsigned> (j)),
                  codeTableP_);
}

void RangingCodeL1OC::initCodePhaseAtSampleL1OC(SampleIndex  globalStartSample,
                                                std::int64_t sampleRate,
                                                double       codePhaseInit) {
   assert(sampleRate        > 0);
   assert(globalStartSample >= 0);                                  // n0 >= 0
   assert(codePhaseInit >= 0.0 && codePhaseInit < multiplexPeriod); // 0 <= phi_{c0,j} < M

   sampleRate_       = sampleRate;
   codePhaseModulus_ = static_cast<std::uint64_t> (multiplexPeriod)
                       * static_cast<std::uint64_t> (sampleRate); // M*Fs в 64 битах (А_L1OC.8)

   // round(phi_{c0,j} * Fs) — half-away-from-zero, совпадает с «round» Ч3
   const std::int64_t initTerm =
      std::llround(codePhaseInit * static_cast<double> (sampleRate));
   // член привязки: n0 * f_T1 mod (M*Fs) без переполнения (А_L1OC.5)
   const std::uint64_t anchorTerm =
      mulMod(static_cast<std::uint64_t> (globalStartSample),
             static_cast<std::uint64_t> (chipRateL1OC),
             codePhaseModulus_);

   codePhaseAccumulator_ =
      (static_cast<std::uint64_t> (initTerm) % codePhaseModulus_ + anchorTerm)
      % codePhaseModulus_; // при phi_{c0,j}=0, n0=0 -> 0
}

int RangingCodeL1OC::multiplexChipIndex() const {
   assert(sampleRate_ > 0);
   // m_j[n] = floor(P_{c,j}[n] / Fs), 0 <= m_j <= M-1 (А_L1OC.6)
   return static_cast<int> (codePhaseAccumulator_ / static_cast<std::uint64_t> (sampleRate_));
}

Bit RangingCodeL1OC::componentSelect() const {
   return static_cast<Bit> (multiplexChipIndex() % 2); // sigma_j[n] = m_j[n] mod 2 (А_L1OC.7)
}

int RangingCodeL1OC::chipIndexD() const {
   return (multiplexChipIndex() / 2) % codeLengthD; // q_{d,j}[n] (А_L1OC.8)
}

int RangingCodeL1OC::chipIndexP() const {
   return (multiplexChipIndex() / 2) % codeLengthP; // q_{p,j}[n] (А_L1OC.8)
}

Bit RangingCodeL1OC::codeBitD() const {
   return codeTableD_[static_cast<std::size_t> (chipIndexD())]; // c_{d,j}[n] (А_L1OC.8)
}

Bit RangingCodeL1OC::codeBitP() const {
   return codeTableP_[static_cast<std::size_t> (chipIndexP())]; // c_{p,j}[n] (А_L1OC.8)
}

Bit RangingCodeL1OC::meanderSymbol() const {
   assert(sampleRate_ > 0);
   const std::uint64_t fs = static_cast<std::uint64_t> (sampleRate_);

   // мп_j[n] = floor(2*(P_{c,j}[n] mod Fs) / Fs) — полусимвол чипа уплотнения (А_L1OC.10);
   // отдельный аккумулятор тактовой 2*f_T1 не вводится (А_L1OC.8)
   return static_cast<Bit> ((2 * (codePhaseAccumulator_ % fs)) / fs);
}

void RangingCodeL1OC::step() {
   // P_{c,j}[n+1] = (P_{c,j}[n] + f_T1) mod (M*Fs) (А_L1OC.9)
   codePhaseAccumulator_ =
      (codePhaseAccumulator_ + static_cast<std::uint64_t> (chipRateL1OC)) % codePhaseModulus_;
}

std::uint64_t RangingCodeL1OC::codePhaseAccumulator() const {
   return codePhaseAccumulator_;
}

const std::array<Bit, codeLengthD> &RangingCodeL1OC::codeTableD() const {
   return codeTableD_;
}

const std::array<Bit, codeLengthP> &RangingCodeL1OC::codeTableP() const {
   return codeTableP_;
}
} // namespace glonass
