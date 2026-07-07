#include "glonass/ranging_code.h"
#include "glonass/code_phase.h"
#include "sha256.h"

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <utility>
#include <fstream>

using namespace glonass;

// --- §7(1) структура таблицы ---
TEST(RangingCodeTable, HeadMatchesIcd) {
   RangingCode code;
   const std::array<Bit, 9> expected{ 1, 1, 1, 1, 1, 1, 1, 0, 0 }; // «111111100» (ИКД)

   for (int q = 0; q < 9; ++q) {
      EXPECT_EQ(code.at(q), expected[q]) << "chipIndex=" << q;
   }
}

TEST(RangingCodeTable, BalanceSumIs256) {
   RangingCode code;
   int sum = 0;

   for (int q = 0; q < codeLength; ++q) {
      sum += code.at(q);
   }
   EXPECT_EQ(sum, 256); // (N+1)/2 — баланс М-последовательности
}

// независимая РСЛОС в тесте совпадает с таблицей ядра,
// и регистр возвращается к [1..1] после 511 тактов (эквивалент s[511]=s[0]).
TEST(RangingCodeTable, IndependentLfsrAndPeriodClosure) {
   RangingCode code;
   std::array<Bit, 9> reg{ 1, 1, 1, 1, 1, 1, 1, 1, 1 };

   for (int q = 0; q < codeLength; ++q) {
      EXPECT_EQ(reg[6], code.at(q)) << "mismatch at q=" << q; // phys 7, съём до сдвига (А.1)
      const Bit fb = static_cast<Bit> (reg[4] ^ reg[8]);      // phys 5,9 (А.2)

      for (int j = 8; j > 0; --j) {
         reg[j] = reg[j - 1];                                 // сдвиг (А.3)
      }
      reg[0] = fb;
   }
   const std::array<Bit, 9> ones{ 1, 1, 1, 1, 1, 1, 1, 1, 1 };
   EXPECT_EQ(reg, ones); // замыкание периода
}

// --- периодическая АКФ: R[0]=511, R[τ]=−1 ∀τ=1..510 ---
TEST(RangingCodeAcf, PerfectPeriodicAutocorrelation) {
   RangingCode code;
   std::array<int, codeLength> chat{}; // Ĉ[q]=1−2·codeTable[q]

   for (int q = 0; q < codeLength; ++q) {
      chat[q] = 1 - 2 * static_cast<int> (code.at(q));
   }

   for (int tau = 0; tau < codeLength; ++tau) {
      long r = 0;

      for (int q = 0; q < codeLength; ++q) {
         r += chat[q] * chat[(q + tau) % codeLength];
      }

      if (tau == 0) {
         EXPECT_EQ(r, 511);
      } else { EXPECT_EQ(r, -1) << "tau=" << tau; }
   }
}

// --- прогрессия кодовой фазы, F=4,096 МГц, φ=0 ---
TEST(CodePhaseProgression, FourMHz_ChipIndex) {
   const std::int64_t Fs = 4096000;
   CodePhase ph; ph.init(/*n0=*/ 0, Fs, /*phi=*/ 0.0);
   const std::array<int, 21> expected{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2 };

   for (int n = 0; n <= 20; ++n) {
      EXPECT_EQ(ph.chipIndex(), expected[n]) << "n=" << n;
      ph.step();
   }
}

TEST(CodePhaseProgression, FourMHz_ChipStarts) {
   const std::int64_t Fs = 4096000;
   CodePhase ph; ph.init(0, Fs, 0.0);
   const std::array<std::pair<int, int>, 6> starts{ { { 0, 0 }, { 1, 9 }, { 2, 17 }, { 3, 25 }, { 4, 33 }, { 5, 41 } } };
   std::size_t s = 0; int prev = -1;

   for (int n = 0; n <= 41 && s < starts.size(); ++n) {
      const int ci = ph.chipIndex();

      if (ci != prev) {
         EXPECT_EQ(ci, starts[s].first);
         EXPECT_EQ(n,  starts[s].second);
         ++s; prev = ci;
      }
      ph.step();
   }
   EXPECT_EQ(s, starts.size());
}

// длительности чипов на периоде: 8×9 + 503×8 = 4096
TEST(CodePhaseProgression, FourMHz_ChipDurations) {
   const std::int64_t Fs = 4096000;
   const int period      = static_cast<int> (codeLength * Fs / codeRate);

   ASSERT_EQ(period, 4096);
   CodePhase ph; ph.init(0, Fs, 0.0);
   std::array<int, codeLength> dur{};

   for (int n = 0; n < period; ++n) {
      ++dur[ph.chipIndex()]; ph.step();
   }
   int n9 = 0, n8 = 0;

   for (int d : dur) {
      if (d == 9) {
         ++n9;
      } else if (d == 8) {
         ++n8;
      } else { FAIL() << "unexpected chip duration " << d; }
   }
   EXPECT_EQ(n9,              8);
   EXPECT_EQ(n8,              503);
   EXPECT_EQ(9 * n9 + 8 * n8, 4096);
   EXPECT_EQ(dur[0],          9);
}

// --- замыкание фазы при F=4,096 МГц ---
TEST(CodePhaseClosure, FourMHz_AccumulatorReturnsToStart) {
   const std::int64_t Fs = 4096000;
   CodePhase ph; ph.init(0, Fs, 0.0);
   const std::uint64_t acc0 = ph.accumulator();

   for (int n = 0; n < 4096; ++n) {
      if (n == 4095) {
         EXPECT_EQ(ph.chipIndex(), 510);
      }
      ph.step();
   }
   EXPECT_EQ(ph.chipIndex(),   0);    // chipIndex(4096)=0
   EXPECT_EQ(ph.accumulator(), acc0); // аккумулятор вернулся к значению n=0
}

// --- прогрессия при F=16,368 МГц: 16×33 + 495×32 = 16368 ---
TEST(CodePhaseProgression, SixteenMHz_ChipDurations) {
   const std::int64_t Fs = 16368000;
   const int period      = static_cast<int> (codeLength * Fs / codeRate);

   ASSERT_EQ(period, 16368);
   CodePhase ph; ph.init(0, Fs, 0.0);
   std::array<int, codeLength> dur{};

   for (int n = 0; n < period; ++n) {
      ++dur[ph.chipIndex()]; ph.step();
   }
   int n33 = 0, n32 = 0;

   for (int d : dur) {
      if (d == 33) {
         ++n33;
      } else if (d == 32) {
         ++n32;
      } else { FAIL() << "unexpected chip duration " << d; }
   }
   EXPECT_EQ(n33,                 16);
   EXPECT_EQ(n32,                 495);
   EXPECT_EQ(33 * n33 + 32 * n32, 16368);
}

TEST(RangingCodeFingerprint, Sha256OfAsciiString) {
   RangingCode code;
   std::string ascii; ascii.reserve(codeLength);

   for (int q = 0; q < codeLength; ++q) {
      ascii.push_back(code.at(q) ? '1' : '0');
   }
   ASSERT_EQ(ascii.size(), static_cast<std::size_t> (codeLength));
   // хеш от ASCII-строки '0'/'1', НЕ от байтов 0x00/0x01 (§7)
   EXPECT_EQ(testutil::Sha256::hexOf(ascii),
             "f9ced31ae34538d0a7040df20f674d55aafde247cbc078680e3fe40a5d50420e");
}

TEST(RangingCodeTable, MatchesGoldenFile) {
#ifndef GLONASS_TEST_DATA_DIR
   GTEST_SKIP() << "GLONASS_TEST_DATA_DIR не определён — файловая сверка пропущена";
#else // ifndef GLONASS_TEST_DATA_DIR
   const std::string path = std::string(GLONASS_TEST_DATA_DIR) + "/code_511_ascii.txt";
   std::ifstream     in(path, std::ios::binary);
   ASSERT_TRUE(in.is_open()) << "не открыт эталон: " << path;
   const std::string golden((std::istreambuf_iterator<char> (in)),
                            std::istreambuf_iterator<char>());
   ASSERT_EQ(golden.size(), static_cast<std::size_t> (codeLength))
      << "длина эталона != " << codeLength;
   RangingCode code;

   for (int q = 0; q < codeLength; ++q) {
      EXPECT_EQ(golden[static_cast<std::size_t> (q)], code.at(q) ? '1' : '0') << "chipIndex=" << q;
   }
#endif // ifndef GLONASS_TEST_DATA_DIR
}
