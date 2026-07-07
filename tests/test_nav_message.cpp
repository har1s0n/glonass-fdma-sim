#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <set>
#include <string>
#include <fstream>

#include "glonass/nav_message.h"
#include "sha256.h"

using namespace glonass;

namespace {
// Единственная точка интеграции с SHA-256 блока А. При ином API — поправьте ТЕЛО этой функции.
std::string sha256Hex(const std::string& ascii) {
   return testutil::Sha256::hexOf(ascii); // <- вызов помощника блока А (tests/sha256.h)
}

// Чтение эталонной строки ASCII '0'/'1' из data/ (GLONASS_TEST_DATA_DIR, как в блоке А).
std::string readGolden(const char* fileName) {
   const std::string path = std::string(GLONASS_TEST_DATA_DIR) + "/" + fileName;
   std::ifstream     f(path);
   std::string s;

   std::getline(f, s);
   return s;
}

// Строка ASCII '0'/'1' из массива (SHA берётся именно от неё, НЕ от байтов 0x00/0x01).
template<std::size_t N>
std::string toAsciiBits(const std::array<Bit, N>& a) {
   std::string s;

   s.reserve(N);

   for (Bit b : a) {
      s.push_back(b ? '1' : '0');
   }
   return s;
}

std::array<Bit, 76> zeroPayload() {
   return std::array<Bit, 76>{};
}

std::array<Bit, 76> unitAt(int position /* 9..84 */) {
   std::array<Bit, 76> p{};

   p[static_cast<std::size_t> (position - 9)] = 1;
   return p;
}

// Синдром одиночной ошибки в позиции p (1..85): (С1..С7, СΣ).
std::array<Bit, 8> syndromeForError(int p) {
   std::array<Bit, 85> rd{};

   rd[static_cast<std::size_t> (p - 1)] ^= 1;        // инверсия позиции p (нулевое слово валидно)
   const std::array<Bit, 8> rec = hammingParity(rd); // пересчитанные β1..β8
   std::array<Bit, 8> syn{};

   for (int r = 0; r < 7; ++r) {                     // С_r = принятое β_r ⊕ пересчитанное β_r
      syn[static_cast<std::size_t> (r)] =
         static_cast<Bit> (rd[static_cast<std::size_t> (r)] ^ rec[static_cast<std::size_t> (r)]);
   }
   Bit cs = 0; // СΣ = чётность всех 85 символов

   for (int i = 0; i < 85; ++i) {
      cs = static_cast<Bit> (cs ^ rd[static_cast<std::size_t> (i)]);
   }
   syn[7] = cs;
   return syn;
}

// Эталонная ПСПМВ по [ИКД] 3.3.2.2 (для теста-провенанса метки времени).
constexpr std::array<Bit, 30> kIcdTimeMark = {
   1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0 };
} // namespace

// --- Тест 1: нулевое поле, a_in=0 -> «1010…10» + метка, a_out=0 (§7(1), Ч3 Б.10) ---
TEST(NavMessageHamming, Test1_ZeroPayloadPattern) {
   const BuiltLine bl = buildLine(zeroPayload(), 0);

   for (int t = 0; t < 85; ++t) {
      EXPECT_EQ(bl.lineSymbols[static_cast<std::size_t> (2 * t)],     1) << "t=" << t;
      EXPECT_EQ(bl.lineSymbols[static_cast<std::size_t> (2 * t + 1)], 0) << "t=" << t;
   }

   for (int m = 0; m < 30; ++m) {
      EXPECT_EQ(bl.lineSymbols[static_cast<std::size_t> (170 + m)], kIcdTimeMark[static_cast<std::size_t> (m)]);
   }
   EXPECT_EQ(bl.relativeStateOut, 0);
}

// --- Тест 2: позиция 9=1, a_in=0 -> β=1,1,0,0,0,0,0,1; SHA; a_out=0 (§7(2)) ---
TEST(NavMessageHamming, Test2_Position9ParityAndSha) {
   std::array<Bit, 85> dPos{};

   dPos[9 - 1] = 1; // поз.9=1, поз.85=0
   const std::array<Bit, 8> beta     = hammingParity(dPos);
   const std::array<Bit, 8> expected = { 1, 1, 0, 0, 0, 0, 0, 1 };
   EXPECT_EQ(beta,                expected);

   const BuiltLine bl = buildLine(unitAt(9), 0);
   EXPECT_EQ(bl.relativeStateOut, 0);
   EXPECT_EQ(sha256Hex(toAsciiBits(bl.lineSymbols)),
             "cf04a58efda962a319c1a0caf19aecfaeb8a8fc579380fcaa922f0ce3164c4a9");
}

// --- Тест 3: непрерывность vs сброс, a_init=1, payload=0 (§7(3)) ---
TEST(NavMessageContinuity, Test3_ContinuousVsReset) {
   // непрерывная модель: a_in2 = a_out1
   const BuiltLine c1 = buildLine(zeroPayload(), 1);

   EXPECT_EQ(c1.relativeStateOut, 1);
   const BuiltLine   c2   = buildLine(zeroPayload(), c1.relativeStateOut);
   const std::string cont = toAsciiBits(c1.lineSymbols) + toAsciiBits(c2.lineSymbols);
   EXPECT_EQ(sha256Hex(cont),
             "651a219147fc33ee4e60ca37c353eaa8d1f836e411961c0f7bae40719a3fff96");

   // сбросовая модель: относительный кодер сбрасывается в 0 в начале каждой строки
   const BuiltLine   r1    = buildLine(zeroPayload(), 0);
   const BuiltLine   r2    = buildLine(zeroPayload(), 0);
   const std::string reset = toAsciiBits(r1.lineSymbols) + toAsciiBits(r2.lineSymbols);
   EXPECT_EQ(sha256Hex(reset),
             "465b8b3bc573a846d8fc9c8caaca3165e75e743c47a6026e03b39f121e92d568");

   EXPECT_NE(sha256Hex(cont), sha256Hex(reset)); // вектор различает кодер и сброс
}

// --- Тест 4: матрица Хэмминга — 85 различных ненулевых синдромов (§7(4)) ---
TEST(NavMessageHamming, Test4_Syndromes85DistinctNonzero) {
   std::set<std::array<Bit, 8> > uniq;
   int nonzero = 0;

   for (int p = 1; p <= 85; ++p) {
      const std::array<Bit, 8> s = syndromeForError(p);
      uniq.insert(s);
      Bit any = 0;

      for (Bit b : s) {
         any = static_cast<Bit> (any | b);
      }

      if (any != 0) {
         ++nonzero;
      }
   }
   EXPECT_EQ(uniq.size(), static_cast<std::size_t> (85)); // все попарно различны
   EXPECT_EQ(nonzero,     85);                            // все ненулевые
}

// --- Тест 5: тактирование Fs=4,096 МГц (§7(5)) ---
TEST(NavMessageTiming, Test5_ClockingAt4096) {
   constexpr std::int64_t Fs = 4096000;

   EXPECT_EQ(Fs / messageRate, 40960); // отсчётов на бидвоичный символ

   NavMessage msg;
   msg.init(0, Fs, [](std::int64_t) {
      return std::array<Bit, 76>{};
   });

   int j0 = -1, j40959 = -1, j40960 = -1, jMark = -1, jLine = -1;
   const std::int64_t nMax = 8192000;         // начало следующей строки (2*Fs)

   for (std::int64_t n = 0; n <= nMax; ++n) {
      const int j = msg.messageSymbolIndex(); // съём ДО обновления

      if (n == 0) {
         j0 = j;
      }

      if (n == 40959) {
         j40959 = j;
      }

      if (n == 40960) {
         j40960 = j;
      }

      if (n == 6963200) {
         jMark = j; // начало метки времени (1,7*Fs)
      }

      if (n == 8192000) {
         jLine = j;
      }

      if (n < nMax) {
         msg.step();
      }
   }
   EXPECT_EQ(j0,                     0);
   EXPECT_EQ(j40959,                 0);
   EXPECT_EQ(j40960,                 1);
   EXPECT_EQ(jMark,                  170);
   EXPECT_EQ(jLine,                  0);
   EXPECT_EQ(msg.lineIndex(),        1); // построена новая строка
   EXPECT_EQ(msg.relativeStateOut(), 0); // payload=0 => a_out=0
}

// --- Тест 6: меандр «начинается с 1» (§7(6)) ---
TEST(NavMessageHamming, Test6_MeanderStartsWithOne) {
   const BuiltLine bl = buildLine(zeroPayload(), 0); // r[0]=0

   EXPECT_EQ(bl.lineSymbols[0], 1);
   EXPECT_EQ(bl.lineSymbols[1], 0);
}

// --- Провенанс метки: генерация РСЛОС g(x)=1+x^3+x^5 воспроизводит явную ПС [ИКД] 3.3.2.2 ---
TEST(NavMessageTimeMark, ProvenanceMatchesIcd) {
   constexpr std::array<Bit, timeMarkLength> generated = detail::buildTimeMarkTable();

   static_assert(generated == kIcdTimeMark, "generated time mark != ICD 3.3.2.2 sequence");
   EXPECT_EQ(generated, kIcdTimeMark);
}

// --- Файловая сверка строк Тестов 1–3 с эталонными артефактами data/ (§7, рекомендательный) ---
TEST(NavMessageGoldenFiles, LinesMatchArtifacts) {
   const BuiltLine t1 = buildLine(zeroPayload(), 0);

   EXPECT_EQ(toAsciiBits(t1.lineSymbols), readGolden("nav_test1_ascii.txt"));

   const BuiltLine t2 = buildLine(unitAt(9), 0);
   EXPECT_EQ(toAsciiBits(t2.lineSymbols), readGolden("nav_test2_ascii.txt"));

   const BuiltLine c1 = buildLine(zeroPayload(), 1);
   const BuiltLine c2 = buildLine(zeroPayload(), c1.relativeStateOut);
   EXPECT_EQ(toAsciiBits(c1.lineSymbols) + toAsciiBits(c2.lineSymbols),
             readGolden("nav_test3_ascii.txt"));
}
