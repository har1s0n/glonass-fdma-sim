// tests/test_iq_convert.cpp — конвертер записи I/Q CF32 LE -> CS16 LE (apps/iq_convert).
// Инструмент сопряжения с внешним приёмником; параметров сигнала не содержит, тракт А–Д
// не затрагивает. Контрольные значения пересчитаны независимо из определений convert.h:
//   k = 32767/max|x|;  q = round(k·x), округление половины от нуля, ограничение ±32767.
#include "convert.h"

#include <array>
#include <cstdint>
#include <gtest/gtest.h>
#include <vector>

namespace {
using glonass_tools::cs16FullScale;

// Байтовые представления float32 LE, выписанные из формата IEEE 754 binary32.
constexpr std::array<unsigned char, 4> bytesPlusOne   = { 0x00, 0x00, 0x80, 0x3F }; //  1,0
constexpr std::array<unsigned char, 4> bytesMinusOne  = { 0x00, 0x00, 0x80, 0xBF }; // −1,0
constexpr std::array<unsigned char, 4> bytesHalf      = { 0x00, 0x00, 0x00, 0x3F }; //  0,5
constexpr std::array<unsigned char, 4> bytesQuarter   = { 0x00, 0x00, 0x80, 0x3E }; //  0,25
constexpr std::array<unsigned char, 4> bytesZero      = { 0x00, 0x00, 0x00, 0x00 }; //  0,0
constexpr std::array<unsigned char, 4> bytesMinusFour = { 0x00, 0x00, 0x80, 0xC0 }; // −4,0
constexpr std::array<unsigned char, 4> bytesTwo       = { 0x00, 0x00, 0x00, 0x40 }; //  2,0
} // namespace

TEST(IqConvertBytes, Test1_ReadFloatLe) {
   EXPECT_FLOAT_EQ(glonass_tools::readFloatLe(bytesPlusOne.data()),    1.0F);
   EXPECT_FLOAT_EQ(glonass_tools::readFloatLe(bytesMinusOne.data()),  -1.0F);
   EXPECT_FLOAT_EQ(glonass_tools::readFloatLe(bytesHalf.data()),       0.5F);
   EXPECT_FLOAT_EQ(glonass_tools::readFloatLe(bytesQuarter.data()),    0.25F);
   EXPECT_FLOAT_EQ(glonass_tools::readFloatLe(bytesZero.data()),       0.0F);
   EXPECT_FLOAT_EQ(glonass_tools::readFloatLe(bytesMinusFour.data()), -4.0F);
   EXPECT_FLOAT_EQ(glonass_tools::readFloatLe(bytesTwo.data()),        2.0F);
}

TEST(IqConvertBytes, Test2_WriteInt16Le) {
   std::array<unsigned char, 2> out{};

   glonass_tools::writeInt16Le(out.data(), 4096); // 0x1000
   EXPECT_EQ(out[0], 0x00);
   EXPECT_EQ(out[1], 0x10);

   glonass_tools::writeInt16Le(out.data(), -1); // 0xFFFF
   EXPECT_EQ(out[0], 0xFF);
   EXPECT_EQ(out[1], 0xFF);

   glonass_tools::writeInt16Le(out.data(), cs16FullScale); // 0x7FFF
   EXPECT_EQ(out[0], 0xFF);
   EXPECT_EQ(out[1], 0x7F);

   glonass_tools::writeInt16Le(out.data(), -cs16FullScale); // 0x8001
   EXPECT_EQ(out[0], 0x01);
   EXPECT_EQ(out[1], 0x80);
}

TEST(IqConvertScale, Test3_ScaleForPeak) {
   EXPECT_DOUBLE_EQ(glonass_tools::scaleForPeakCs16(4.0), 8191.75); // 32767/4
   EXPECT_DOUBLE_EQ(glonass_tools::scaleForPeakCs16(1.0), 32767.0);
   EXPECT_DOUBLE_EQ(glonass_tools::scaleForPeakCs16(0.0), 1.0);     // нулевая запись
}

TEST(IqConvertQuantize, Test4_RoundHalfAwayFromZero) {
   EXPECT_EQ(glonass_tools::quantizeCs16(0.5F,  1.0), 1);
   EXPECT_EQ(glonass_tools::quantizeCs16(-0.5F, 1.0), -1);
   EXPECT_EQ(glonass_tools::quantizeCs16(1.5F,  1.0), 2);
   EXPECT_EQ(glonass_tools::quantizeCs16(-1.5F, 1.0), -2);
   EXPECT_EQ(glonass_tools::quantizeCs16(2.5F,  1.0), 3);
   EXPECT_EQ(glonass_tools::quantizeCs16(0.4F,  1.0), 0);
   EXPECT_EQ(glonass_tools::quantizeCs16(-0.4F, 1.0), 0);
}

TEST(IqConvertQuantize, Test5_SymmetricClamp) {
   EXPECT_EQ(glonass_tools::quantizeCs16(2.0F,  20000.0), cs16FullScale);  // 40000 -> 32767
   EXPECT_EQ(glonass_tools::quantizeCs16(-2.0F, 20000.0), -cs16FullScale);
}

// Масштаб под пик записи по построению выводит пик ровно на полную шкалу.
TEST(IqConvertScale, Test6_PeakMapsToFullScale) {
   const double peak  = 4.898979485566356; // |J|·η при |J| = 24, η = 1/√24 — пик записи Д_L1OC.10
   const double scale = glonass_tools::scaleForPeakCs16(peak);

   EXPECT_EQ(glonass_tools::quantizeCs16(static_cast<float> (peak), scale), cs16FullScale);
   EXPECT_EQ(glonass_tools::quantizeCs16(static_cast<float> (-peak), scale), -cs16FullScale);
}

TEST(IqConvertBlock, Test7_PeakAbsOfBlock) {
   // 2 отсчёта: (1,0; −1,0) и (0,5; −4,0) — пик 4,0 находится в координате Q
   std::vector<unsigned char> in;

   for (const auto& b : { bytesPlusOne, bytesMinusOne, bytesHalf, bytesMinusFour }) {
      in.insert(in.end(), b.begin(), b.end());
   }
   EXPECT_DOUBLE_EQ(glonass_tools::peakAbsOfBlock(in.data(), 2), 4.0);
   EXPECT_DOUBLE_EQ(glonass_tools::peakAbsOfBlock(in.data(), 1), 1.0); // только первый отсчёт
}

// Композиция всего пути: 4 отсчёта CF32 -> 4 отсчёта CS16 при k = 32767/4 = 8191,75.
// Ожидаемые коды: 1,0 -> round(8191,75) = 8192;   −1,0 -> −8192;   0,5 -> round(4095,875) = 4096;
//                 0,25 -> round(2047,9375) = 2048; 0,0 -> 0;
//                 −4,0 -> round(−32767) = −32767;  2,0 -> round(16383,5) = 16384.
TEST(IqConvertBlock, Test8_ConvertBlockCs16) {
   std::vector<unsigned char> in;

   for (const auto& b : { bytesPlusOne, bytesMinusOne,  // отсчёт 0: I = 1,0;  Q = −1,0
                          bytesHalf,    bytesQuarter,   // отсчёт 1: I = 0,5;  Q = 0,25
                          bytesZero,    bytesZero,      // отсчёт 2: I = 0,0;  Q = 0,0
                          bytesMinusFour, bytesTwo }) { // отсчёт 3: I = −4,0; Q = 2,0
      in.insert(in.end(), b.begin(), b.end());
   }
   const std::size_t sampleCount = 4;
   const double      peak        = glonass_tools::peakAbsOfBlock(in.data(), sampleCount);

   ASSERT_DOUBLE_EQ(peak, 4.0);
   const double scale = glonass_tools::scaleForPeakCs16(peak);

   std::vector<unsigned char> out(sampleCount * 4);
   glonass_tools::convertBlockCs16(in.data(), sampleCount, scale, out.data());

   const std::vector<unsigned char> expected = {
      0x00, 0x20, 0x00, 0xE0, // 8192, −8192
      0x00, 0x10, 0x00, 0x08, // 4096, 2048
      0x00, 0x00, 0x00, 0x00, // 0, 0
      0x01, 0x80, 0x00, 0x40  // −32767, 16384
   };

   EXPECT_EQ(out, expected);
}
