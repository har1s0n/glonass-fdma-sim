#ifndef SERVICE_SVG_CANVAS_H
#define SERVICE_SVG_CANVAS_H

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

// Шаблон кадра
//
// Единый макет для всех кадров: холст 960×540 с viewBox, поля и зоны, стек шрифтов без
// внешних файлов, русские подписи, запятая и узкий неразрывный пробел в числах, деления из
// ряда 1–2–5, двухуровневая сетка, светлая тема
namespace glonass_service {
namespace svg {
inline constexpr int width        = 960;
inline constexpr int height       = 540;
inline constexpr int marginLeft   = 64;
inline constexpr int marginRight  = 24;
inline constexpr int marginTop    = 84;
inline constexpr int marginBottom = 66;

inline constexpr int plotLeft   = marginLeft;
inline constexpr int plotRight  = width - marginRight;
inline constexpr int plotTop    = marginTop;
inline constexpr int plotBottom = height - marginBottom;

inline constexpr const char* background  = "#ffffff";
inline constexpr const char* textPrimary = "#10151c";
inline constexpr const char* textSecond  = "#55606e";
inline constexpr const char* gridMajor   = "#dfe4ea";
inline constexpr const char* gridMinor   = "#f0f3f6";
inline constexpr const char* axisColor   = "#98a2b0";

inline constexpr const char* series1   = "#123f8f";
inline constexpr const char* series2   = "#c2410c";
inline constexpr const char* series3   = "#0f766e";
inline constexpr const char* markColor = "#6b21a8"; // опорные линии и зоны

inline constexpr const char* dashSolid  = "";
inline constexpr const char* dashLine   = "7 4";
inline constexpr const char* dashDotted = "2 3";
inline constexpr const char* dashMark   = "9 3 2 3";
inline constexpr const char* dashFill   = "fill"; // образец заливки зоны в легенде
} // namespace svg

// Русский числовой формат: запятая как десятичный разделитель, узкий неразрывный пробел
// в разрядах, знак «минус» U+2212.
std::string numberRu(double value,
                     int    digits = 0);

std::string escapeXml(const std::string& text);

inline constexpr std::size_t compositionListLimit = 4; // до скольких НКА состав перечисляется

// Состав активных НКА в шапке кадра: до compositionListLimit — перечислением, далее мощностью
// множества
std::string compositionOf(const std::vector<int>& satellites);

// Угол поля графика, в котором размещается легенда
enum class LegendCorner { rightTop, rightBottom, leftTop, leftBottom };

struct LegendRow {
   const char* color = svg::series1;
   const char* dash  = svg::dashSolid; // svg::dashFill — образец заливки
   std::string label;
};

// Бин ряда после прореживания: центр и границы значений в бине
struct Bin {
   double center = 0.0;
   double low    = 0.0;
   double high   = 0.0;
};

class SvgCanvas {
public:

   SvgCanvas(std::string              title,
             std::vector<std::string> subtitles,
             std::string              xLabel,
             std::string              yLabel,
             std::string              description);

   // Границы оси задаются точно: диапазон данных не расширяется. Шаг — из ряда 1–2–5;
   // при digits < 0 число знаков подписи выводится из шага.
   SvgCanvas &setX(double lo,
                   double hi,
                   double step,
                   int    minor  = 5,
                   int    digits = -1);
   SvgCanvas &setY(double lo,
                   double hi,
                   double step,
                   int    minor  = 5,
                   int    digits = -1);

   SvgCanvas &band(double      x0,
                   double      x1,
                   const char* color   = svg::markColor,
                   double      opacity = 0.05);
   SvgCanvas &verticalLine(double             x,
                           const char*        color     = svg::markColor,
                           const char*        dash      = svg::dashMark,
                           const std::string& label     = {},
                           bool               atTop     = false,
                           bool               anchorEnd = false);
   SvgCanvas &horizontalLine(double             y,
                             const char*        color     = svg::markColor,
                             const char*        dash      = svg::dashMark,
                             const std::string& label     = {},
                             bool               anchorEnd = true);
   SvgCanvas &polyline(const std::vector<std::pair<double, double> >& points,
                       const char*                                    color       = svg::series1,
                       double                                         strokeWidth = 1.6,
                       const char*                                    dash        = svg::dashSolid);
   SvgCanvas &steps(const std::vector<std::pair<double, double> >& points,
                    const char*                                    color       = svg::series1,
                    double                                         strokeWidth = 1.6);
   SvgCanvas &envelope(const std::vector<Bin>& bins,
                       const char*             color   = svg::series1,
                       double                  opacity = 0.18);
   SvgCanvas &bars(const std::vector<double>& edges,
                   const std::vector<double>& values,
                   const char*                color   = svg::series1,
                   double                     opacity = 0.8);
   SvgCanvas &rectangle(double      x0,
                        double      x1,
                        double      y0,
                        double      y1,
                        const char* color,
                        double      opacity = 0.3,
                        bool        stroke  = true);
   SvgCanvas &labelAt(double             x,
                      double             y,
                      const std::string& text,
                      int                size   = 11,
                      const char*        color  = svg::textPrimary,
                      const char*        anchor = "middle");
   SvgCanvas &note(double             xPixels,
                   double             yPixels,
                   const std::string& text,
                   int                size   = 12,
                   const char*        color  = svg::textPrimary,
                   const char*        anchor = "start");
   SvgCanvas & legend(std::vector<LegendRow> rows,
                      LegendCorner           corner = LegendCorner::rightTop);

   std::string str() const;

private:

   double      fx(double x) const;
   double      fy(double y) const;
   std::string textElement(double             x,
                           double             y,
                           const std::string& text,
                           int                size,
                           const char*        color,
                           const char*        weight = "400",
                           const char*        anchor = "start",
                           const std::string& extra  = {}) const;
   void appendGrid(std::string& out) const;
   void appendLegend(std::string& out) const;

   std::string title_;
   std::vector<std::string> subtitles_;
   std::string xLabel_;
   std::string yLabel_;
   std::string description_;

   double xLo_ = 0.0, xHi_ = 1.0, xStep_ = 1.0;
   double yLo_ = 0.0, yHi_ = 1.0, yStep_ = 1.0;
   int xMinor_ = 5, yMinor_ = 5, xDigits_ = 0, yDigits_ = 0;

   std::vector<std::string> body_;    // содержимое поля графика (отсекается по границам)
   std::vector<std::string> overlay_; // подписи поверх поля
   std::vector<LegendRow> legendRows_;
   LegendCorner legendCorner_ = LegendCorner::rightTop;
};

// Шаг деления из ряда 1–2–5 × 10^k при заданном числе делений
double niceStep(double span,
                int    target = 6);
} // namespace glonass_service

#endif // SERVICE_SVG_CANVAS_H
