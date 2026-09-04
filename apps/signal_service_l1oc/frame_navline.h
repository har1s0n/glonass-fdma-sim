#ifndef SERVICE_FRAME_NAVLINE_H
#define SERVICE_FRAME_NAVLINE_H

#include <cstdint>
#include <string>
#include <vector>

#include "glonass/nav_message_l1oc.h"
#include "stream_session.h"

// Кадр строки навигационного сообщения (kind = navline)
//
// Наблюдаемый блок: Б_L1OC информационный блок «СМВ + ЦИ» -> циклический код -> свёрточный
// код (133,171) и привязка момента n₀ к позиции в строке
namespace glonass_service {
inline constexpr int navlineSmvBits            = 12;   // СМВ — биты 1…12 строки ([ИКД-L1OC] 4.2.2.1)
inline constexpr double navlineAxisLimit       = 1.5;  // граница оси символов: запас над ±1
inline constexpr double navlineAxisStep        = 0.5;
inline constexpr double navlineAxisStepSymbols = 50.0; // шаг оси символов СК: 50 симв. = 200 мс
inline constexpr double navlineFieldLabelLevel = 1.25; // уровень подписей полей по оси Y

// Поле строки на оси символов СК: полуинтервал [from; to) и его длина в битах строки
struct NavLineField {
   double      from = 0.0;
   double      to   = 0.0;
   int         bits = 0;
   const char* name = "";
};

struct NavLineFrame {
   std::int64_t              lineIndex    = 0;                             // № строки потока (Б_L1OC.9)
   glonass::LineTypeL1OC     lineType     = glonass::LineTypeL1OC::normal; // тип строки (поз.37)
   int                       infoBits     = 0;                             // бит ЦИ
   int                       lineBitCount = 0;                             // n_с
   int                       lineLength   = 0;                             // L_с = 2·n_с
   std::vector<int>          lineSymbols;                                  // b_line[0…L_с−1] ∈ {0, 1} (Б_L1OC.6)
   std::vector<NavLineField> fields;                                       // зоны полей: СМВ, ЦИ, ЦК
   int                       convSymbolIndex = 0;                          // w[n₀] (Б_L1OC.8)
   double                    symbolPhase     = 0.0;                        // P_s[n₀]/Fs — доля символа СК
   glonass::ConvStateL1OC    convStateOut{};                               // S после строки (Б_L1OC.5)
   int                       onesCount        = 0;                         // единиц среди символов СК
   int                       transitions      = 0;                         // смен значения между соседними символами
   double                    symbolDurationMs = 0.0;                       // 1000/R_с
   double                    lineDurationS    = 0.0;                       // L_с/R_с
};

NavLineFrame computeNavLineFrame(const StreamRequest& request);

std::string  navLineFrameJson(const NavLineFrame&  frame,
                              const StreamRequest& request);
std::string  navLineFrameSvg(const NavLineFrame&  frame,
                             const StreamRequest& request);
} // namespace glonass_service

#endif // SERVICE_FRAME_NAVLINE_H
