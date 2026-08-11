#include "glonass/ranging_code_l1oc.h"
#include "sha256.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#ifndef GLONASS_TEST_DATA_DIR
#error "GLONASS_TEST_DATA_DIR не определён -> сверка с эталонами ИКД невозможна"
#endif // ifndef GLONASS_TEST_DATA_DIR

using namespace glonass;

namespace {
constexpr std::int64_t sampleRateL1OC = 20000000; // Fs = 20,0 МГц (поз.35)
constexpr int samplesPer8ms           = 160000;   // 8 мс при Fs = 20,0 МГц (А_L1OC.10)

// --- независимая реализация РСЛОС (соглашение поз.31), написана отдельно от ядра ---
class TestLfsr {
public:

   TestLfsr(const std::string& initialState, std::vector<int> taps)
      : taps_(std::move(taps)) {
      for (char c : initialState) {
         s_.push_back(static_cast<Bit> (c - '0')); // слева направо на триггеры 1…L
      }
   }

   Bit out() const {
      return s_.back(); // съём с последнего триггера ДО сдвига
   }

   void tick() {
      Bit f = 0;

      for (int t : taps_) {
         f = static_cast<Bit> (f ^ s_[static_cast<std::size_t> (t - 1)]); // ОС до сдвига
      }
      s_.insert(s_.begin(), f);
      s_.pop_back();
   }

   const std::vector<Bit> &state() const {
      return s_;
   }

private:

   std::vector<Bit> s_;
   std::vector<int> taps_;
};

// двоичная запись j разрядности length, старший разряд слева (НС2)
std::string binaryOf(int j, int length) {
   std::string out(static_cast<std::size_t> (length), '0');

   for (int t = 1; t <= length; ++t) {
      out[static_cast<std::size_t> (t - 1)] = ((j >> (length - t)) & 1) ? '1' : '0';
   }
   return out;
}

// ДК = ЦА1 XOR ЦА2 на count тактов БЕЗ перезаписи НС; параметры продублированы
// в тесте независимо от ядра ([ИКД-L1OC] 2.2.1 рис.2.4, 2.2.2 рис.2.5)
std::vector<Bit> independentCodeD(int j, int count) {
   TestLfsr ca1("0011001000",   { 7, 10 });
   TestLfsr ca2(binaryOf(j, 10), { 3, 7, 9, 10 });
   std::vector<Bit> out;

   for (int m = 0; m < count; ++m) {
      out.push_back(static_cast<Bit> (ca1.out() ^ ca2.out()));
      ca1.tick(); ca2.tick();
   }
   return out;
}

std::vector<Bit> independentCodeP(int j, int count) {
   TestLfsr ca1("000011000101", { 6, 8, 11, 12 });
   TestLfsr ca2(binaryOf(j, 6),  { 1, 6 });
   std::vector<Bit> out;

   for (int m = 0; m < count; ++m) {
      out.push_back(static_cast<Bit> (ca1.out() ^ ca2.out()));
      ca1.tick(); ca2.tick();
   }
   return out;
}

// 32 символа -> 8 hex; первый по времени символ — старший разряд ([ИКД-L1OC] 2.2.1)
template<std::size_t N>
std::string hex32(const std::array<Bit, N>& table, std::size_t offset) {
   std::uint32_t v = 0;

   for (std::size_t i = 0; i < 32; ++i) {
      v = (v << 1) | static_cast<std::uint32_t> (table[offset + i]);
   }
   static const char* digits = "0123456789ABCDEF";
   std::string out(8, '0');

   for (int i = 0; i < 8; ++i) {
      out[static_cast<std::size_t> (i)] = digits[(v >> (28 - 4 * i)) & 0xFu];
   }
   return out;
}

template<std::size_t N>
std::string asciiOf(const std::array<Bit, N>& table) {
   std::string out; out.reserve(N);

   for (std::size_t q = 0; q < N; ++q) {
      out.push_back(table[q] ? '1' : '0');
   }
   return out;
}

// периодическая АКФ: R[0] и гистограмма значений R[tau], tau = 1…N-1
template<std::size_t N>
std::map<int, int> acfHistogram(const std::array<Bit, N>& table, int& peak) {
   std::array<int, N> chat{};

   for (std::size_t q = 0; q < N; ++q) {
      chat[q] = 1 - 2 * static_cast<int> (table[q]);
   }
   peak = 0;

   for (std::size_t q = 0; q < N; ++q) {
      peak += chat[q] * chat[q];
   }
   std::map<int, int> hist;

   for (std::size_t tau = 1; tau < N; ++tau) {
      int r = 0;

      for (std::size_t q = 0; q < N; ++q) {
         r += chat[q] * chat[(q + tau) % N];
      }
      ++hist[r];
   }
   return hist;
}

std::vector<std::string> splitCsv(const std::string& line) {
   std::vector<std::string> out;
   std::string cell;
   std::istringstream in(line);

   while (std::getline(in, cell, ',')) {
      if (!cell.empty() && (cell.back() == '\r')) {
         cell.pop_back();
      }
      out.push_back(cell);
   }
   return out;
}

std::vector<std::vector<std::string> > loadCsv(const std::string& path) {
   std::ifstream in(path);

   EXPECT_TRUE(in.is_open()) << "не открыт эталон: " << path;
   std::vector<std::vector<std::string> > rows;
   std::string line;
   bool header = true;

   while (std::getline(in, line)) {
      if (line.empty()) {
         continue;
      }

      if (header) {
         header = false; continue;
      }
      rows.push_back(splitCsv(line));
   }
   return rows;
}

std::string dataPath(const char* name) {
   return std::string(GLONASS_TEST_DATA_DIR) + "/" + name;
}

RangingCodeL1OC makeCode(int j, SampleIndex globalStartSample = 0, double codePhaseInit = 0.0) {
   RangingCodeL1OC code;

   code.initCodeTablesL1OC(j);
   code.initCodePhaseAtSampleL1OC(globalStartSample, sampleRateL1OC, codePhaseInit);
   return code;
}
} // namespace

// --- А_L1OC.10(1): сверка с таблицами 2.1 и 2.2 ИКД ---
TEST(RangingCodeL1OCTables, IcdTable21FirstLast32) {
   const auto rows = loadCsv(dataPath("dk_l1ocd_ref.csv"));

   ASSERT_EQ(rows.size(), static_cast<std::size_t> (satelliteCount));

   for (const auto& row : rows) {
      ASSERT_EQ(row.size(), 4u);
      const int j = std::stoi(row[0]);
      RangingCodeL1OC code;
      code.initCodeTablesL1OC(j);
      const auto& table = code.codeTableD();
      EXPECT_EQ(row[1],                                                    binaryOf(j, 10)) << "НС2, j=" << j;
      EXPECT_EQ(hex32(table, 0),                                           row[2]) << "первые 32, j=" << j;
      EXPECT_EQ(hex32(table, static_cast<std::size_t> (codeLengthD - 32)), row[3])
         << "последние 32, j=" << j;
   }
}

TEST(RangingCodeL1OCTables, IcdTable22FirstLast32) {
   const auto rows = loadCsv(dataPath("dk_l1ocp_ref.csv"));

   ASSERT_EQ(rows.size(), static_cast<std::size_t> (satelliteCount));

   for (const auto& row : rows) {
      ASSERT_EQ(row.size(), 4u);
      const int j = std::stoi(row[0]);
      RangingCodeL1OC code;
      code.initCodeTablesL1OC(j);
      const auto& table = code.codeTableP();
      EXPECT_EQ(row[1],                                                    binaryOf(j, 6)) << "НС2, j=" << j;
      EXPECT_EQ(hex32(table, 0),                                           row[2]) << "первые 32, j=" << j;
      EXPECT_EQ(hex32(table, static_cast<std::size_t> (codeLengthP - 32)), row[3])
         << "последние 32, j=" << j;
   }
}

// --- А_L1OC.10(2): при j = 0 автомат ЦА2 нулевой => первые L символов = реверс НС1 ---
TEST(RangingCodeL1OCTables, ZeroSatelliteHeadIsReversedNs1) {
   RangingCodeL1OC code;

   code.initCodeTablesL1OC(0);
   std::string headD, headP;

   for (int q = 0; q < 12; ++q) {
      headD.push_back(code.codeTableD()[static_cast<std::size_t> (q)] ? '1' : '0');
      headP.push_back(code.codeTableP()[static_cast<std::size_t> (q)] ? '1' : '0');
   }
   EXPECT_EQ(headD, "000100110010"); // НС1 = 0011001000 -> реверс 0001001100
   EXPECT_EQ(headP, "101000110000"); // НС1 = 000011000101 -> реверс 101000110000
}

// --- независимый РСЛОС в тесте совпадает с таблицами ядра ---
TEST(RangingCodeL1OCTables, IndependentLfsrMatchesTables) {
   for (int j : { 0, 1, 2, 63 }) {
      RangingCodeL1OC code;
      code.initCodeTablesL1OC(j);
      const std::vector<Bit> refD = independentCodeD(j, codeLengthD);
      const std::vector<Bit> refP = independentCodeP(j, codeLengthP);

      for (int q = 0; q < codeLengthD; ++q) {
         ASSERT_EQ(code.codeTableD()[static_cast<std::size_t> (q)],
                   refD[static_cast<std::size_t> (q)]) << "L1OCd j=" << j << " q=" << q;
      }

      for (int q = 0; q < codeLengthP; ++q) {
         ASSERT_EQ(code.codeTableP()[static_cast<std::size_t> (q)],
                   refP[static_cast<std::size_t> (q)]) << "L1OCp j=" << j << " q=" << q;
      }
   }
}

// --- А_L1OC.4: перезапись НС каждые N тактов. Для L1OCd ненаблюдаема (период ЦА1 = 1023),
// для L1OCp содержательна (естественный период ЦА1 = 4095 != 4092) ---
TEST(RangingCodeL1OCTables, PeriodClosureAndTruncation) {
   for (int j : { 0, 1, 63 }) {
      const std::vector<Bit> contD = independentCodeD(j, 2 * codeLengthD);
      const std::vector<Bit> contP = independentCodeP(j, 2 * codeLengthP);
      bool equalD = true, equalP = true;

      for (int q = 0; q < codeLengthD; ++q) {
         equalD = equalD
                  && contD[static_cast<std::size_t> (q)]
                  == contD[static_cast<std::size_t> (q + codeLengthD)];
      }

      for (int q = 0; q < codeLengthP; ++q) {
         equalP = equalP
                  && contP[static_cast<std::size_t> (q)]
                  == contP[static_cast<std::size_t> (q + codeLengthP)];
      }
      EXPECT_TRUE(equalD) << "L1OCd j=" << j << ": период 1023 должен замыкаться";
      EXPECT_FALSE(equalP) << "L1OCp j=" << j << ": усечение 4095->4092 должно разрушать период";
   }
}

// --- А_L1OC.10(3): SHA-256 таблиц по всем 64 НКА обеих компонент ---
TEST(RangingCodeL1OCTables, Sha256AllSatellites) {
   const auto rows = loadCsv(dataPath("sha256_codes.csv"));

   ASSERT_EQ(rows.size(), static_cast<std::size_t> (2 * satelliteCount));

   for (const auto& row : rows) {
      ASSERT_EQ(row.size(), 4u);
      const std::string component = row[0];
      const int j                 = std::stoi(row[1]);
      const int n                 = std::stoi(row[2]);
      RangingCodeL1OC code;
      code.initCodeTablesL1OC(j);

      if (component == "L1OCd") {
         ASSERT_EQ(n, codeLengthD);
         EXPECT_EQ(testutil::Sha256::hexOf(asciiOf(code.codeTableD())), row[3])
            << "L1OCd j=" << j;
      } else {
         ASSERT_EQ(component, "L1OCp");
         ASSERT_EQ(n,         codeLengthP);
         EXPECT_EQ(testutil::Sha256::hexOf(asciiOf(code.codeTableP())), row[3])
            << "L1OCp j=" << j;
      }
   }
}

// --- А_L1OC.10(4): ПАКФ ДК_L1OCd трёхзначная ---
TEST(RangingCodeL1OCAcf, L1OCdThreeValued) {
   const std::map<int, std::map<int, int> > expected{
      { 0,  { { -1, 1022 } } },
      { 1,  { { -65, 112 }, { -1, 798 }, { 63, 112 } } },
      { 2,  { { -65, 144 }, { -1, 734 }, { 63, 144 } } },
      { 32, { { -65, 100 }, { -1, 822 }, { 63, 100 } } },
      { 63, { { -65, 96 },  { -1, 768 }, { 63, 158 } } }
   };

   for (const auto& item : expected) {
      RangingCodeL1OC code;
      code.initCodeTablesL1OC(item.first);
      int peak        = 0;
      const auto hist = acfHistogram(code.codeTableD(), peak);
      EXPECT_EQ(peak, codeLengthD) << "R(0), j=" << item.first;
      EXPECT_EQ(hist, item.second) << "гистограмма R(tau), j=" << item.first;
   }
}

// дешёвый инвариант по всему ансамблю: N_d - 2*sum(ДК) in {-65, -1, +63} (А_L1OC.10)
TEST(RangingCodeL1OCAcf, L1OCdBalanceInvariant) {
   for (int j = 0; j < satelliteCount; ++j) {
      RangingCodeL1OC code;
      code.initCodeTablesL1OC(j);
      int sum = 0;

      for (int q = 0; q < codeLengthD; ++q) {
         sum += code.codeTableD()[static_cast<std::size_t> (q)];
      }
      const int balance = codeLengthD - 2 * sum;
      EXPECT_TRUE(balance == -65 || balance == -1 || balance == 63)
         << "j=" << j << " баланс=" << balance;
   }
}

// --- А_L1OC.10(5): ПАКФ ДК_L1OCp — главный пик и максимальный боковой лепесток ---
TEST(RangingCodeL1OCAcf, L1OCpMainPeakAndSideLobes) {
   const std::map<int, int> expectedMax{ { 0, 108 }, { 1, 204 }, { 63, 188 } };

   for (const auto& item : expectedMax) {
      RangingCodeL1OC code;
      code.initCodeTablesL1OC(item.first);
      int peak        = 0;
      const auto hist = acfHistogram(code.codeTableP(), peak);
      EXPECT_EQ(peak, codeLengthP) << "R(0), j=" << item.first;
      int maxAbs = 0;

      for (const auto& bin : hist) {
         maxAbs = std::max(maxAbs, bin.first < 0 ? -bin.first : bin.first);
      }
      EXPECT_EQ(maxAbs, item.second) << "max|R(tau)|, j=" << item.first;
   }
}

// --- А_L1OC.10(6): прогрессия кодовой фазы при Fs = 20,0 МГц, phi = 0, n0 = 0 ---
TEST(CodePhaseL1OCProgression, TwentyMHzChipIndex) {
   RangingCodeL1OC code = makeCode(1);

   for (int n = 0; n <= 40; ++n) {
      const int expected = (n < 20) ? 0 : (n < 40 ? 1 : 2);
      EXPECT_EQ(code.multiplexChipIndex(), expected) << "n=" << n;
      EXPECT_EQ(code.componentSelect(),    static_cast<Bit> (expected % 2)) << "n=" << n;
      code.step();
   }
}

TEST(CodePhaseL1OCProgression, TwentyMHzChipStartsAndDurations) {
   RangingCodeL1OC code = makeCode(1);
   const std::array<std::pair<int, int>, 8> starts{
      { { 0, 0 }, { 1, 20 }, { 2, 40 }, { 3, 59 }, { 4, 79 }, { 5, 98 }, { 6, 118 }, { 7, 137 } }
   };
   std::size_t s = 0; int prev = -1;

   for (int n = 0; n <= 137 && s < starts.size(); ++n) {
      const int m = code.multiplexChipIndex();

      if (m != prev) {
         EXPECT_EQ(m, starts[s].first);
         EXPECT_EQ(n, starts[s].second) << "начало чипа m=" << m;
         ++s; prev = m;
      }
      code.step();
   }
   ASSERT_EQ(s, starts.size());
   const std::array<int, 7> durations{ 20, 20, 19, 20, 19, 20, 19 };

   for (std::size_t i = 0; i < durations.size(); ++i) {
      EXPECT_EQ(starts[i + 1].second - starts[i].second, durations[i]) << "чип " << i;
   }
}

// --- А_L1OC.10(7): чередование компонент и замыкание кодовой фазы на 8 мс ---
TEST(CodePhaseL1OCClosure, ComponentAlternationOver8ms) {
   RangingCodeL1OC code     = makeCode(1);
   const std::uint64_t acc0 = code.codePhaseAccumulator();
   std::vector<int>    visitsD(codeLengthD, 0), visitsP(codeLengthP, 0);
   int chips = 0, chipsD = 0, chipsP = 0, prev = -1;

   for (int n = 0; n < samplesPer8ms; ++n) {
      const int m = code.multiplexChipIndex();

      if (m != prev) {
         ++chips;

         if (code.componentSelect() == 0) {
            ++chipsD; ++visitsD[static_cast<std::size_t> (code.chipIndexD())];
         } else {
            ++chipsP; ++visitsP[static_cast<std::size_t> (code.chipIndexP())];
         }
         prev = m;
      }
      code.step();
   }
   EXPECT_EQ(chips,  multiplexPeriod);                                        // M = 8184 чипов уплотнения на 8 мс
   EXPECT_EQ(chipsD, codeLengthP);                                            // 4092 чипа компоненты L1OCd
   EXPECT_EQ(chipsP, codeLengthP);                                            // 4092 чипа компоненты L1OCp

   for (int q = 0; q < codeLengthD; ++q) {
      ASSERT_EQ(visitsD[static_cast<std::size_t> (q)], 4) << "L1OCd q=" << q; // 4 полных периода
   }

   for (int q = 0; q < codeLengthP; ++q) {
      ASSERT_EQ(visitsP[static_cast<std::size_t> (q)], 1) << "L1OCp q=" << q; // один период
   }
   EXPECT_EQ(code.codePhaseAccumulator(), acc0);                              // P_c[160000] = P_c[0] = 0
   EXPECT_EQ(code.multiplexChipIndex(),   0);
}

// --- А_L1OC.5: тождество формы МП закрытому виду 0101 на 160 000 отсчётов ---
TEST(CodePhaseL1OCMeander, IdentityToIcdPatternOver8ms) {
   RangingCodeL1OC code = makeCode(1);
   // независимый аккумулятор тактовой 2*f_T1; индекс полусимвола 0…2M-1
   const std::uint64_t halfChipModulus =
      2ull * static_cast<std::uint64_t> (multiplexPeriod) * static_cast<std::uint64_t> (sampleRateL1OC);
   std::uint64_t meanderPhase = 0;
   // МП = 0101, четыре символа на символ ДК_L1OCp, первый по времени — «0» ([ИКД-L1OC] 2.1.3)
   const std::array<Bit, 4> meanderPattern{ 0, 1, 0, 1 };
   int mismatches = 0, firstBad = -1;

   for (int n = 0; n < samplesPer8ms; ++n) {
      const int halfSymbolIndex =
         static_cast<int> (meanderPhase / static_cast<std::uint64_t> (sampleRateL1OC));

      if (code.meanderSymbol() != meanderPattern[static_cast<std::size_t> (halfSymbolIndex % 4)]) {
         ++mismatches;

         if (firstBad < 0) {
            firstBad = n;
         }
      }
      meanderPhase = (meanderPhase + 2ull * static_cast<std::uint64_t> (chipRateL1OC))
                     % halfChipModulus;
      code.step();
   }
   EXPECT_EQ(mismatches, 0) << "первое расхождение при n=" << firstBad;
}

// --- А_L1OC.5: привязка к n0 и калибровочное смещение phi_{c0,j} ---
TEST(CodePhaseL1OCInit, AnchorAtSampleMatchesStepping) {
   const SampleIndex n0      = 12345;
   RangingCodeL1OC   stepped = makeCode(1);

   for (SampleIndex n = 0; n < n0; ++n) {
      stepped.step();
   }
   RangingCodeL1OC anchored = makeCode(1, n0);
   EXPECT_EQ(anchored.codePhaseAccumulator(), stepped.codePhaseAccumulator());
   EXPECT_EQ(anchored.multiplexChipIndex(),   stepped.multiplexChipIndex());
   EXPECT_EQ(anchored.codeBitD(),             stepped.codeBitD());
   EXPECT_EQ(anchored.codeBitP(),             stepped.codeBitP());
   EXPECT_EQ(anchored.meanderSymbol(),        stepped.meanderSymbol());
}

TEST(CodePhaseL1OCInit, CalibrationOffsetShiftsChip) {
   RangingCodeL1OC shifted = makeCode(1, /*n0=*/ 0, /*phi=*/ 1.0); // смещение на один чип уплотнения

   EXPECT_EQ(shifted.codePhaseAccumulator(), static_cast<std::uint64_t> (sampleRateL1OC));
   EXPECT_EQ(shifted.multiplexChipIndex(),   1);
   EXPECT_EQ(shifted.componentSelect(),      1); // нечётный чип — L1OCp
}
