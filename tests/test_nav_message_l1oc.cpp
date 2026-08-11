#include "glonass/nav_message_l1oc.h"
#include "sha256.h"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

using namespace glonass;

namespace {
constexpr std::int64_t sampleRateL1OC = 20000000; // Fs = 20,0 МГц

// --- независимая реализация СК: отводы выводятся из восьмеричных полиномов (133,171) ---
// ([ИКД-L1OC] 2.3); ядро задаёт те же отводы явными суммами (Б_L1OC.3), (Б_L1OC.4).
constexpr unsigned kGFirst  = 0133u;
constexpr unsigned kGSecond = 0171u;

// индексы слагаемых ветви: 0 — вход u, t = 1…6 — триггер S[t]
std::vector<int> tapsFromOctal(unsigned g) {
   std::vector<int> taps;

   for (int i = 0; i < 7; ++i) {
      if ((g >> (6 - i)) & 1u) {
         taps.push_back(i);
      }
   }
   return taps;
}

// вклад состояния в символ ветви (слагаемые без входа u)
Bit stateContribution(unsigned g, const ConvStateL1OC& s) {
   Bit acc = 0;

   for (int i : tapsFromOctal(g)) {
      if (i != 0) {
         acc = static_cast<Bit> (acc ^ s[static_cast<std::size_t> (i - 1)]);
      }
   }
   return acc;
}

struct ConvResult {
   std::vector<Bit> symbols;
   ConvStateL1OC    state{};
};

ConvResult convEncode(const std::vector<Bit>& bits, ConvStateL1OC state,
                      unsigned gFirst = kGFirst, unsigned gSecond = kGSecond) {
   ConvResult result;

   result.symbols.reserve(2 * bits.size());

   for (Bit u : bits) {
      result.symbols.push_back(static_cast<Bit> (u ^ stateContribution(gFirst,  state)));
      result.symbols.push_back(static_cast<Bit> (u ^ stateContribution(gSecond, state)));

      for (int tau = 5; tau > 0; --tau) {
         state[static_cast<std::size_t> (tau)] = state[static_cast<std::size_t> (tau - 1)];
      }
      state[0] = u;
   }
   result.state = state;
   return result;
}

// Восстановление бит строки d[1…n_с] из символов СК при известном состоянии на входе строки.
// Даёт доступ к выходу циклического кодера, который построитель строки наружу не выдаёт.
std::vector<Bit> decodeLineBits(const std::array<Bit, maxLineSymbolsL1OC>& symbols, int lineLength,
                                ConvStateL1OC state) {
   std::vector<Bit> bits;

   for (int t = 0; t < lineLength / 2; ++t) {
      const Bit v1 = symbols[static_cast<std::size_t> (2 * t)];
      const Bit v2 = symbols[static_cast<std::size_t> (2 * t + 1)];
      const Bit u  = static_cast<Bit> (v1 ^ stateContribution(kGFirst, state));

      EXPECT_EQ(v2, static_cast<Bit> (u ^ stateContribution(kGSecond, state))) << "t=" << t;
      bits.push_back(u);

      for (int tau = 5; tau > 0; --tau) {
         state[static_cast<std::size_t> (tau)] = state[static_cast<std::size_t> (tau - 1)];
      }
      state[0] = u;
   }
   return bits;
}

// --- независимая проверка ЦК: деление многочленов над GF(2) ---
// степени ненулевых коэффициентов образующих полиномов (§ 0.1 поз.42; [ИКД-L1OC] 4.4–4.6)
const std::vector<int> kG250 = { 0, 1, 5, 6, 8, 9, 10, 11, 13, 14, 16 };
const std::vector<int> kG375 = { 0, 1, 3, 4, 5, 6, 7, 10, 11, 14, 17, 18, 23, 24 };

const std::vector<int> &generatorOf(LineTypeL1OC lineType) {
   return lineType == LineTypeL1OC::anomalous2 ? kG375 : kG250;
}

// остаток m(X)·X^L mod g(X); первый бит информационного блока — старший коэффициент
std::vector<Bit> polyRemainder(const std::vector<Bit>& info, const std::vector<int>& gdeg) {
   const int L     = gdeg.back();
   std::uint64_t g = 0;

   for (int d : gdeg) {
      g |= (1ull << d);
   }
   std::uint64_t r = 0;

   for (std::size_t i = 0; i < info.size() + static_cast<std::size_t> (L); ++i) {
      const std::uint64_t b = (i < info.size()) ? info[i] : 0ull; // умножение на X^L
      r = (r << 1) | b;

      if ((r >> L) & 1ull) {
         r ^= g;
      }
   }
   std::vector<Bit> out;

   for (int i = L - 1; i >= 0; --i) {
      out.push_back(static_cast<Bit> ((r >> i) & 1ull));
   }
   return out;
}

// c(X) mod g(X) для собранной строки; инвариант — нуль (Б_L1OC.4(1))
std::uint64_t codeBlockRemainder(const std::vector<Bit>& line, const std::vector<int>& gdeg) {
   const int L     = gdeg.back();
   std::uint64_t g = 0;

   for (int d : gdeg) {
      g |= (1ull << d);
   }
   std::uint64_t r = 0;

   for (Bit b : line) {
      r = (r << 1) | b;

      if ((r >> L) & 1ull) {
         r ^= g;
      }
   }
   return r;
}

// Схема вычисления синдрома ([ИКД-L1OC] рисунки 4.5, 4.7): первые L сдвигов обратная связь
// разомкнута, далее замкнута; принятый бит заносится в триггер 1.
std::vector<Bit> syndromeOf(const std::vector<Bit>& received, const std::vector<int>& gdeg) {
   const int L = gdeg.back();
   std::set<int> taps(gdeg.begin() + 1, gdeg.end() - 1);
   std::vector<Bit> s(static_cast<std::size_t> (L + 1), 0); // s[1…L]

   for (std::size_t idx = 0; idx < received.size(); ++idx) {
      const Bit fb = (idx >= static_cast<std::size_t> (L)) ? s[static_cast<std::size_t> (L)]
                                                           : static_cast<Bit> (0);
      std::vector<Bit> next(static_cast<std::size_t> (L + 1), 0);
      next[1] = static_cast<Bit> (received[idx] ^ fb);

      for (int t = 1; t <= L - 1; ++t) {
         const Bit addend = taps.count(t) ? fb : static_cast<Bit> (0);
         next[static_cast<std::size_t> (t + 1)] =
            static_cast<Bit> (s[static_cast<std::size_t> (t)] ^ addend);
      }
      s = next;
   }
   return std::vector<Bit> (s.begin() + 1, s.end());
}

// --- сервис ---
std::string asciiOf(const std::vector<Bit>& bits) {
   std::string out;

   out.reserve(bits.size());

   for (Bit b : bits) {
      out.push_back(b ? '1' : '0');
   }
   return out;
}

std::string asciiOf(const std::array<Bit, maxLineSymbolsL1OC>& symbols, int lineLength) {
   std::string out;

   out.reserve(static_cast<std::size_t> (lineLength));

   for (int i = 0; i < lineLength; ++i) {
      out.push_back(symbols[static_cast<std::size_t> (i)] ? '1' : '0');
   }
   return out;
}

std::string hexOf(const std::vector<Bit>& bits) {
   static const char* digits = "0123456789ABCDEF";
   std::string   out;
   std::uint64_t v = 0;

   for (Bit b : bits) {
      v = (v << 1) | b;
   }
   const int nibbles = static_cast<int> ((bits.size() + 3) / 4);

   for (int i = nibbles - 1; i >= 0; --i) {
      out.push_back(digits[(v >> (4 * i)) & 0xFull]);
   }
   return out;
}

// СМВ = 010111110001 (поз.40; [ИКД-L1OC] 4.2.2.1) — независимо от таблицы ядра
const std::vector<Bit> kSmv = { 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1 };

// тестовые заполнения ЦИ (Б_L1OC.11: детерминированные, кодово корректные)
LineContentL1OC zeroPayload(LineTypeL1OC lineType) {
   LineContentL1OC c;

   c.lineType = lineType;
   return c;
}

LineContentL1OC alternatingPayload(LineTypeL1OC lineType) {
   LineContentL1OC c = zeroPayload(lineType);

   for (int i = 0; i < maxPayloadBitsL1OC; ++i) {
      c.payloadCI[static_cast<std::size_t> (i)] = static_cast<Bit> (1 - (i % 2)); // 1010…
   }
   return c;
}

LineContentL1OC unitFirstBitPayload(LineTypeL1OC lineType) {
   LineContentL1OC c = zeroPayload(lineType);

   c.payloadCI[0] = 1;
   return c;
}

// информационный блок «СМВ + ЦИ» для независимого деления
std::vector<Bit> infoBlockOf(const LineContentL1OC& content) {
   std::vector<Bit> info = kSmv;

   for (int i = 0; i < lineInfoBits(content.lineType); ++i) {
      info.push_back(content.payloadCI[static_cast<std::size_t> (i)]);
   }
   return info;
}

// проверочные биты, восстановленные из строки, построенной ядром
std::vector<Bit> parityFromCore(const LineContentL1OC& content) {
   const BuiltLineL1OC built   = buildLineL1OC(content, ConvStateL1OC{});
   const std::vector<Bit> line = decodeLineBits(built.lineSymbols, built.lineLength, ConvStateL1OC{});
   const std::size_t k         = infoBlockOf(content).size();

   return std::vector<Bit> (line.begin() + static_cast<std::ptrdiff_t> (k), line.end());
}

PayloadProviderL1OC constantProvider(const LineContentL1OC& content) {
   return [content](std::int64_t) {
             return content;
   };
}
} // namespace

// --- Тест 1: соглашение по ветвям СК — символы 12…23 инвариантны (Ч3 Б_L1OC.10, поз.39) ---
TEST(NavMessageL1OCConvCoder, Test1_SyncSymbolsInvariantOverAllStates) {
   std::set<std::string> firstTwelve, syncTwelve;

   for (int st = 0; st < 64; ++st) {
      ConvStateL1OC state{};

      for (int t = 0; t < 6; ++t) {
         state[static_cast<std::size_t> (t)] = static_cast<Bit> ((st >> t) & 1);
      }
      const BuiltLineL1OC built = buildLineL1OC(zeroPayload(LineTypeL1OC::normal), state);
      const std::string   line  = asciiOf(built.lineSymbols, 24);

      firstTwelve.insert(line.substr(0, 12));
      syncTwelve.insert(line.substr(12, 12));
   }
   EXPECT_EQ(syncTwelve.size(),   static_cast<std::size_t> (1));
   EXPECT_EQ(*syncTwelve.begin(), "000111011010"); // [ИКД-L1OC] 4.2.2.1
   EXPECT_EQ(firstTwelve.size(),  static_cast<std::size_t> (64));
}

// --- Тест 2: обратный порядок ветвей контрольному значению не отвечает (Ч3 Б_L1OC.10) ---
TEST(NavMessageL1OCConvCoder, Test2_ReversedBranchOrderRejected) {
   const ConvResult direct   = convEncode(kSmv, ConvStateL1OC{});
   const ConvResult reversed = convEncode(kSmv, ConvStateL1OC{}, kGSecond, kGFirst);

   const std::vector<Bit> directSync(direct.symbols.begin() + 12,   direct.symbols.end());
   const std::vector<Bit> reversedSync(reversed.symbols.begin() + 12, reversed.symbols.end());

   EXPECT_EQ(asciiOf(directSync),   "000111011010");
   EXPECT_EQ(asciiOf(reversedSync), "001011100101");
   EXPECT_NE(asciiOf(directSync), asciiOf(reversedSync));
}

// --- Тест 3: нулевое состояние — 24 символа СМВ и состояние после СМВ (Ч3 Б_L1OC.10) ---
TEST(NavMessageL1OCConvCoder, Test3_ZeroStateSmvSymbols) {
   const BuiltLineL1OC built = buildLineL1OC(zeroPayload(LineTypeL1OC::normal), ConvStateL1OC{});

   EXPECT_EQ(asciiOf(built.lineSymbols, 24), "001101000101000111011010");

   const ConvResult independent = convEncode(kSmv, ConvStateL1OC{});
   const ConvStateL1OC expected = { 1, 0, 0, 0, 1, 1 }; // S[1..6] после СМВ

   EXPECT_EQ(asciiOf(independent.symbols), "001101000101000111011010");
   EXPECT_EQ(independent.state,            expected);
}

// --- Тест 4: ЦК трёх типов строк, ЦИ нулевая (Ч3 Б_L1OC.10) ---
TEST(NavMessageL1OCCyclicCode, Test4_ZeroPayloadParity) {
   EXPECT_EQ(hexOf(parityFromCore(zeroPayload(LineTypeL1OC::normal))),     "4EB4");
   EXPECT_EQ(hexOf(parityFromCore(zeroPayload(LineTypeL1OC::anomalous1))), "350D");
   EXPECT_EQ(hexOf(parityFromCore(zeroPayload(LineTypeL1OC::anomalous2))), "D2BFD8");
}

// --- Тест 5: ЦК трёх типов строк, ЦИ = 1010… (Ч3 Б_L1OC.10) ---
TEST(NavMessageL1OCCyclicCode, Test5_AlternatingPayloadParity) {
   EXPECT_EQ(hexOf(parityFromCore(alternatingPayload(LineTypeL1OC::normal))),     "8E5C");
   EXPECT_EQ(hexOf(parityFromCore(alternatingPayload(LineTypeL1OC::anomalous1))), "F271");
   EXPECT_EQ(hexOf(parityFromCore(alternatingPayload(LineTypeL1OC::anomalous2))), "4FDDFC");
}

// --- Тест 6: ЦК при одиночной «1» в первом бите ЦИ (независимый расчёт gate_l1oc_message) ---
TEST(NavMessageL1OCCyclicCode, Test6_UnitFirstBitParity) {
   EXPECT_EQ(hexOf(parityFromCore(unitFirstBitPayload(LineTypeL1OC::normal))),     "3E66");
   EXPECT_EQ(hexOf(parityFromCore(unitFirstBitPayload(LineTypeL1OC::anomalous1))), "83AE");
   EXPECT_EQ(hexOf(parityFromCore(unitFirstBitPayload(LineTypeL1OC::anomalous2))), "288575");
}

// --- Тест 7: ЦК ядра совпадает с делением m(X)·X^L на g(X); d(X) mod g(X) = 0 (Ч3 Б_L1OC.10) ---
TEST(NavMessageL1OCCyclicCode, Test7_DivisionInvariant) {
   const LineTypeL1OC types[] = { LineTypeL1OC::normal, LineTypeL1OC::anomalous1, LineTypeL1OC::anomalous2 };

   for (LineTypeL1OC type : types) {
      const LineContentL1OC contents[] = { zeroPayload(type), alternatingPayload(type),
                                           unitFirstBitPayload(type) };

      for (const LineContentL1OC& content : contents) {
         const BuiltLineL1OC built   = buildLineL1OC(content, ConvStateL1OC{});
         const std::vector<Bit> line = decodeLineBits(built.lineSymbols, built.lineLength, ConvStateL1OC{});

         ASSERT_EQ(static_cast<int> (line.size()), lineBits(type));
         EXPECT_EQ(parityFromCore(content),                     polyRemainder(infoBlockOf(content), generatorOf(type)));
         EXPECT_EQ(codeBlockRemainder(line, generatorOf(type)), 0ull);
      }
   }
}

// --- Тест 8: синдром по схеме [ИКД-L1OC] рис. 4.5 — нулевой; при одиночной ошибке ненулевой ---
TEST(NavMessageL1OCCyclicCode, Test8_SyndromeDetectsSingleError) {
   const BuiltLineL1OC built    = buildLineL1OC(zeroPayload(LineTypeL1OC::normal), ConvStateL1OC{});
   std::vector<Bit>    line     = decodeLineBits(built.lineSymbols, built.lineLength, ConvStateL1OC{});
   const std::vector<Bit> clean = syndromeOf(line, kG250);

   EXPECT_EQ(asciiOf(clean),                   std::string(16, '0'));

   line[100] = static_cast<Bit> (line[100] ^ 1); // одиночная ошибка в бите 101
   EXPECT_EQ(asciiOf(syndromeOf(line, kG250)), "0111000000101010");
   EXPECT_EQ(codeBlockRemainder(line, kG250),  21518ull);
}

// --- Тест 9: SHA-256 строк трёх типов, ЦИ нулевая, состояние СК нулевое (Ч3 Б_L1OC.10) ---
TEST(NavMessageL1OCLine, Test9_Sha256OfThreeLineTypes) {
   const BuiltLineL1OC normal     = buildLineL1OC(zeroPayload(LineTypeL1OC::normal),     ConvStateL1OC{});
   const BuiltLineL1OC anomalous1 = buildLineL1OC(zeroPayload(LineTypeL1OC::anomalous1), ConvStateL1OC{});
   const BuiltLineL1OC anomalous2 = buildLineL1OC(zeroPayload(LineTypeL1OC::anomalous2), ConvStateL1OC{});

   EXPECT_EQ(normal.lineLength,     500);
   EXPECT_EQ(anomalous1.lineLength, 250);
   EXPECT_EQ(anomalous2.lineLength, 750);

   EXPECT_EQ(testutil::Sha256::hexOf(asciiOf(normal.lineSymbols, normal.lineLength)),
             "c33412bc4c013056c079ad0c74ec1d54afba19aea20729ce90da44440ae862df");
   EXPECT_EQ(testutil::Sha256::hexOf(asciiOf(anomalous1.lineSymbols, anomalous1.lineLength)),
             "ce63a5040d833d28c2c254cbb90c45b6992310eedbd8a7a542224342d5b5bf39");
   EXPECT_EQ(testutil::Sha256::hexOf(asciiOf(anomalous2.lineSymbols, anomalous2.lineLength)),
             "1ea8bcde7af46ae8b84f9db5447e0e84a56e0d2d3e0f0b2044090a5158c5988e");

   EXPECT_EQ(asciiOf(anomalous1.lineSymbols, 24), "001101000101000111011010");
   EXPECT_EQ(asciiOf(anomalous2.lineSymbols, 24), "001101000101000111011010");
}

// --- Тест 10: непрерывность СК между строками против сброса (Ч3 Б_L1OC.10, поз.41) ---
TEST(NavMessageL1OCLine, Test10_ContinuousEncoderVsReset) {
   const BuiltLineL1OC first         = buildLineL1OC(zeroPayload(LineTypeL1OC::normal), ConvStateL1OC{});
   const ConvStateL1OC expectedState = { 0, 0, 1, 0, 1, 1 };

   EXPECT_EQ(first.convStateOut, expectedState);

   const BuiltLineL1OC continuous = buildLineL1OC(zeroPayload(LineTypeL1OC::normal), first.convStateOut);
   const BuiltLineL1OC reset      = buildLineL1OC(zeroPayload(LineTypeL1OC::normal), ConvStateL1OC{});

   EXPECT_EQ(asciiOf(continuous.lineSymbols, 12),            "100011110101");
   EXPECT_EQ(asciiOf(reset.lineSymbols,      12),            "001101000101");
   EXPECT_EQ(asciiOf(continuous.lineSymbols, 24).substr(12), "000111011010"); // синхропоследовательность
   EXPECT_EQ(asciiOf(reset.lineSymbols,      24).substr(12), "000111011010");

   const std::string continuousPair = asciiOf(first.lineSymbols, first.lineLength)
                                      + asciiOf(continuous.lineSymbols, continuous.lineLength);
   const std::string resetPair = asciiOf(first.lineSymbols, first.lineLength)
                                 + asciiOf(reset.lineSymbols, reset.lineLength);

   EXPECT_EQ(testutil::Sha256::hexOf(continuousPair),
             "f8b3580820c27312e921efdaf7590ede7add8f29ef9adc8b700b0271bb073fb1");
   EXPECT_EQ(testutil::Sha256::hexOf(resetPair),
             "a13ffa3b00d4d8d06bc22533efb41a7350910e6575cf8bf467e4b7ff014e77f6");
   EXPECT_NE(testutil::Sha256::hexOf(continuousPair), testutil::Sha256::hexOf(resetPair));
}

// --- Тест 11: тактирование при Fs = 20,0 МГц — ОК1 и индекс символа (Ч3 Б_L1OC.10) ---
TEST(NavMessageL1OCTiming, Test11_OverlayAndSymbolIndex) {
   EXPECT_EQ(sampleRateL1OC / symbolRateL1OC, 80000); // отсчётов на символ СК
   EXPECT_EQ(sampleRateL1OC / 500,            40000); // отсчётов на символ ОК1 (R_ок = 500 Гц)

   NavMessageL1OC msg;

   msg.initMessageAtSampleL1OC(0, sampleRateL1OC, constantProvider(zeroPayload(LineTypeL1OC::normal)));

   int overlay0 = -1, overlay39999 = -1, overlay40000 = -1, overlay79999 = -1, overlay80000 = -1;
   int w0 = -1, w79999 = -1, w80000 = -1;

   for (std::int64_t n = 0; n <= 80000; ++n) {
      const Bit o = msg.overlaySymbol(); // съём ДО обновления (Б_L1OC.7)
      const int w = msg.convSymbolIndex();

      if (n == 0) {
         overlay0 = o; w0     = w;
      }

      if (n == 39999) {
         overlay39999 = o;
      }

      if (n == 40000) {
         overlay40000 = o;
      }

      if (n == 79999) {
         overlay79999 = o; w79999 = w;
      }

      if (n == 80000) {
         overlay80000 = o; w80000 = w;
      }
      msg.step();
   }
   EXPECT_EQ(overlay0,     0);
   EXPECT_EQ(overlay39999, 0);
   EXPECT_EQ(overlay40000, 1);
   EXPECT_EQ(overlay79999, 1);
   EXPECT_EQ(overlay80000, 0);
   EXPECT_EQ(w0,           0);
   EXPECT_EQ(w79999,       0);
   EXPECT_EQ(w80000,       1);
}

// --- Тест 12: граница строки на n = 40 000 000 (2 с) с переносом состояния СК (Ч3 Б_L1OC.8) ---
TEST(NavMessageL1OCTiming, Test12_LineBoundaryAtTwoSeconds) {
   NavMessageL1OC msg;

   msg.initMessageAtSampleL1OC(0, sampleRateL1OC, constantProvider(zeroPayload(LineTypeL1OC::normal)));

   const ConvStateL1OC stateAfterFirstLine = msg.convStateOut();
   const ConvStateL1OC expectedState       = { 0, 0, 1, 0, 1, 1 };

   EXPECT_EQ(stateAfterFirstLine, expectedState);

   const std::int64_t samplesPerLine = 40000000; // 500 символов × 80 000 отсчётов

   for (std::int64_t n = 0; n < samplesPerLine - 1; ++n) {
      msg.step();
   }
   EXPECT_EQ(msg.convSymbolIndex(), 499); // последний символ первой строки
   EXPECT_EQ(msg.lineIndex(),       0);

   msg.step();                            // переход w: 499 → 0
   EXPECT_EQ(msg.convSymbolIndex(),        0);
   EXPECT_EQ(msg.lineIndex(),              1);
   EXPECT_EQ(msg.lineLength(),             500);
   EXPECT_EQ(msg.symbolPhaseAccumulator(), 0ull);

   // первый символ второй строки отвечает непрерывному кодеру («100011110101»), не сбросу
   const BuiltLineL1OC continuous = buildLineL1OC(zeroPayload(LineTypeL1OC::normal), stateAfterFirstLine);

   EXPECT_EQ(msg.convSymbol(), continuous.lineSymbols[0]);
   EXPECT_EQ(msg.convSymbol(), 1);
}

// --- Тест 13: координаты запуска из n₀ (Ч3 Б_L1OC.4(4)) ---
TEST(NavMessageL1OCInit, Test13_CoordinatesFromStartSample) {
   struct Case {
      SampleIndex   n0;
      std::int64_t  lineIndex;
      int           convSymbolIndex;
      std::uint64_t symbolPhase;
   };
   const Case cases[] = {
      {           0, 0,    0,      0ull                    },
      {           1, 0,    0,      250ull                  },
      {    20000000, 0,    250,    0ull                    },
      {    39999999, 0,    499,    19999750ull             },
      {    40000000, 1,    0,      0ull                    },
      { 60000000000, 1500, 0,      0ull                    },
   };

   for (const Case& c : cases) {
      NavMessageL1OC msg;

      msg.initMessageAtSampleL1OC(c.n0, sampleRateL1OC,
                                  constantProvider(zeroPayload(LineTypeL1OC::normal)));
      EXPECT_EQ(msg.lineIndex(),              c.lineIndex) << "n0=" << c.n0;
      EXPECT_EQ(msg.convSymbolIndex(),        c.convSymbolIndex) << "n0=" << c.n0;
      EXPECT_EQ(msg.symbolPhaseAccumulator(), c.symbolPhase) << "n0=" << c.n0;
      EXPECT_EQ(msg.lineLength(),             500) << "n0=" << c.n0;
   }

   // индекс строки передаётся слою содержания (Б_L1OC.11)
   std::int64_t   requested = -1;
   NavMessageL1OC msg;

   msg.initMessageAtSampleL1OC(40000000, sampleRateL1OC, [&requested](std::int64_t lineIndex) {
      requested = lineIndex;
      return zeroPayload(LineTypeL1OC::normal);
   });
   EXPECT_EQ(requested, 1);

   // при n₀ = 39 999 999 отсчёт лежит на второй половине символа СК: o = 1 (Б_L1OC.9)
   NavMessageL1OC late;

   late.initMessageAtSampleL1OC(39999999, sampleRateL1OC,
                                constantProvider(zeroPayload(LineTypeL1OC::normal)));
   EXPECT_EQ(late.overlaySymbol(), 1);
}

// --- Тест 14: смена типа строки — длина L_с обновляется на границе строки (Ч3 § 2_L1OC.4) ---
TEST(NavMessageL1OCLineTypes, Test14_LineLengthFollowsType) {
   constexpr std::int64_t testRate            = 1000; // 4 отсчёта на символ СК — прогон без длинных циклов
   const std::array<LineTypeL1OC, 4> sequence = { LineTypeL1OC::normal,     LineTypeL1OC::anomalous1,
                                                  LineTypeL1OC::anomalous2, LineTypeL1OC::normal };

   NavMessageL1OC msg;

   msg.initMessageAtSampleL1OC(0, testRate, [sequence](std::int64_t lineIndex) {
      const std::size_t idx = static_cast<std::size_t> (lineIndex) % 4;
      return zeroPayload(sequence[idx]);
   });

   const int expectedLength[] = { 500, 250, 750, 500 };
   std::int64_t lineIndex     = 0;

   for (int line = 0; line < 4; ++line) {
      EXPECT_EQ(msg.lineIndex(),  lineIndex) << "строка " << line;
      EXPECT_EQ(msg.lineLength(), expectedLength[line]) << "строка " << line;

      // отсчётов в строке = L_с * Fs / R_с = L_с * 4
      const std::int64_t samplesInLine = static_cast<std::int64_t> (expectedLength[line]) * 4;

      for (std::int64_t n = 0; n < samplesInLine; ++n) {
         msg.step();
      }
      lineIndex += 1;
   }
   EXPECT_EQ(msg.lineIndex(),       4);
   EXPECT_EQ(msg.lineLength(),      500); // строка 4: 4 mod 4 = 0 → нормальная
   EXPECT_EQ(msg.convSymbolIndex(), 0);
}

// --- Тест 15: двухфазная дисциплина — выходы неизменны до step() (Ч3 Б_L1OC.7) ---
TEST(NavMessageL1OCTwoPhase, Test15_OutputsStableUntilStep) {
   NavMessageL1OC msg;

   msg.initMessageAtSampleL1OC(0, sampleRateL1OC, constantProvider(zeroPayload(LineTypeL1OC::normal)));

   for (int repeat = 0; repeat < 5; ++repeat) { // повторный съём на том же n не изменяет состояния
      EXPECT_EQ(msg.convSymbol(),             msg.convSymbol());
      EXPECT_EQ(msg.overlaySymbol(),          msg.overlaySymbol());
      EXPECT_EQ(msg.convSymbolIndex(),        0);
      EXPECT_EQ(msg.symbolPhaseAccumulator(), 0ull);
   }

   const Bit symbolAtZero = msg.convSymbol();

   msg.step();
   EXPECT_EQ(msg.symbolPhaseAccumulator(), static_cast<std::uint64_t> (symbolRateL1OC));
   EXPECT_EQ(symbolAtZero,                 0); // b_line[0] = 0 (первый символ ветви 133)
}
