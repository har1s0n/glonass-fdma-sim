#include "svg_canvas.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace glonass_service {
namespace {
constexpr const char* fontStack = "\"Helvetica Neue\", Helvetica, Arial, sans-serif";
constexpr const char* minusSign = "\xE2\x88\x92";  // U+2212
constexpr const char* thinSpace = "\xE2\x80\xAF";  // U+202F, узкий неразрывный пробел

std::string format(const char* pattern, double value) {
   char buffer[64];

   std::snprintf(buffer, sizeof(buffer), pattern, value);
   return buffer;
}

// Число знаков подписи деления по величине шага
int digitsForStep(double step) {
   if (step >= 1.0) {
      return 0;
   }
   return std::max(0, static_cast<int> (std::ceil(-std::log10(step) - 1e-9)));
}
} // namespace

double niceStep(double span, int target) {
   const double raw = span / std::max(target, 1);

   if (!(raw > 0.0)) {
      return 1.0;
   }
   const double power = std::floor(std::log10(raw));
   const double base  = raw / std::pow(10.0, power);
   const double step  = (base <= 1.5) ? 1.0 : (base <= 3.5) ? 2.0 : (base <= 7.5) ? 5.0 : 10.0;

   return step * std::pow(10.0, power);
}

std::string numberRu(double value, int digits) {
   const bool negative = value < 0.0;
   std::string text    = format(("%." + std::to_string(std::max(digits, 0)) + "f").c_str(),
                                std::fabs(value));
   const std::size_t point = text.find('.');
   std::string whole       = (point == std::string::npos) ? text : text.substr(0, point);
   const std::string frac  = (point == std::string::npos) ? std::string() : text.substr(point + 1);
   std::string grouped;

   for (std::size_t i = 0; i < whole.size(); ++i) {
      if ((i > 0) && (((whole.size() - i) % 3) == 0)) {
         grouped += thinSpace;
      }
      grouped += whole[i];
   }
   std::string out = (negative ? minusSign : "") + grouped;

   if (!frac.empty()) {
      out += ",";
      out += frac;
   }
   return out;
}

std::string escapeXml(const std::string& text) {
   std::string out;

   out.reserve(text.size());

   for (const char symbol : text) {
      switch (symbol) {
        case '&':  out += "&amp;";  break;
        case '<':  out += "&lt;";   break;
        case '>':  out += "&gt;";   break;
        case '"':  out += "&quot;"; break;
        default:   out += symbol;   break;
      }
   }
   return out;
}

SvgCanvas::SvgCanvas(std::string title, std::vector<std::string> subtitles,
                     std::string xLabel, std::string yLabel, std::string description)
   : title_(std::move(title)), subtitles_(std::move(subtitles)), xLabel_(std::move(xLabel)),
   yLabel_(std::move(yLabel)), description_(std::move(description)) {}

SvgCanvas &SvgCanvas::setX(double lo, double hi, double step, int minor, int digits) {
   xLo_    = lo;
   xHi_    = (hi > lo) ? hi : (lo + 1.0);
   xStep_  = (step > 0.0) ? step : niceStep(xHi_ - xLo_);
   xMinor_ = std::max(minor, 1);
   xDigits_ = (digits >= 0) ? digits : digitsForStep(xStep_);
   return *this;
}

SvgCanvas &SvgCanvas::setY(double lo, double hi, double step, int minor, int digits) {
   yLo_    = lo;
   yHi_    = (hi > lo) ? hi : (lo + 1.0);
   yStep_  = (step > 0.0) ? step : niceStep(yHi_ - yLo_);
   yMinor_ = std::max(minor, 1);
   yDigits_ = (digits >= 0) ? digits : digitsForStep(yStep_);
   return *this;
}

double SvgCanvas::fx(double x) const {
   const double span = xHi_ - xLo_;

   return svg::plotLeft + (x - xLo_) / span * (svg::plotRight - svg::plotLeft);
}

double SvgCanvas::fy(double y) const {
   const double span = yHi_ - yLo_;

   return svg::plotBottom - (y - yLo_) / span * (svg::plotBottom - svg::plotTop);
}

std::string SvgCanvas::textElement(double x, double y, const std::string& text, int size,
                                   const char* color, const char* weight, const char* anchor,
                                   const std::string& extra) const {
   char head[256];

   std::snprintf(head, sizeof(head),
                 "<text x=\"%.1f\" y=\"%.1f\" font-family='%s' font-size=\"%d\" fill=\"%s\" "
                 "font-weight=\"%s\" text-anchor=\"%s\"", x, y, fontStack, size, color, weight,
                 anchor);
   return std::string(head) + extra + ">" + escapeXml(text) + "</text>";
}

SvgCanvas &SvgCanvas::band(double x0, double x1, const char* color, double opacity) {
   char element[256];

   std::snprintf(element, sizeof(element),
                 "<rect x=\"%.1f\" y=\"%d\" width=\"%.1f\" height=\"%d\" fill=\"%s\" "
                 "fill-opacity=\"%.3f\"/>", fx(x0), svg::plotTop, fx(x1) - fx(x0),
                 svg::plotBottom - svg::plotTop, color, opacity);
   body_.emplace_back(element);
   return *this;
}

SvgCanvas &SvgCanvas::verticalLine(double x, const char* color, const char* dash,
                                   const std::string& label, bool atTop, bool anchorEnd) {
   char element[256];

   std::snprintf(element, sizeof(element),
                 "<line x1=\"%.1f\" y1=\"%d\" x2=\"%.1f\" y2=\"%d\" stroke=\"%s\" "
                 "stroke-width=\"1.4\" stroke-dasharray=\"%s\"/>", fx(x), svg::plotTop, fx(x),
                 svg::plotBottom, color, dash);
   body_.emplace_back(element);

   if (!label.empty()) {
      const double y  = atTop ? (svg::plotTop + 14) : (svg::plotBottom - 12);
      const double dx = anchorEnd ? -6.0 : 6.0;

      overlay_.push_back(textElement(fx(x) + dx, y, label, 11, color, "400",
                                     anchorEnd ? "end" : "start"));
   }
   return *this;
}

SvgCanvas &SvgCanvas::horizontalLine(double y, const char* color, const char* dash,
                                     const std::string& label, bool anchorEnd) {
   char element[256];

   std::snprintf(element, sizeof(element),
                 "<line x1=\"%d\" y1=\"%.1f\" x2=\"%d\" y2=\"%.1f\" stroke=\"%s\" "
                 "stroke-width=\"1.4\" stroke-dasharray=\"%s\"/>", svg::plotLeft, fy(y),
                 svg::plotRight, fy(y), color, dash);
   body_.emplace_back(element);

   if (!label.empty()) {
      const double x = anchorEnd ? (svg::plotRight - 8) : (svg::plotLeft + 8);

      overlay_.push_back(textElement(x, fy(y) - 6, label, 11, color, "400",
                                     anchorEnd ? "end" : "start"));
   }
   return *this;
}

SvgCanvas &SvgCanvas::polyline(const std::vector<std::pair<double, double> >& points,
                               const char* color, double strokeWidth, const char* dash) {
   if (points.empty()) {
      return *this;
   }
   std::string element = "<polyline points=\"";

   for (const auto& [x, y] : points) {
      element += format("%.1f", fx(x)) + "," + format("%.1f", fy(y)) + " ";
   }
   element += "\" fill=\"none\" stroke=\"";
   element += color;
   element += "\" stroke-width=" + format("\"%.2f\"", strokeWidth) + " stroke-linejoin=\"round\"";

   if ((dash != nullptr) && (dash[0] != '\0')) {
      element += std::string(" stroke-dasharray=\"") + dash + "\"";
   }
   element += "/>";
   body_.push_back(std::move(element));
   return *this;
}

SvgCanvas &SvgCanvas::steps(const std::vector<std::pair<double, double> >& points,
                            const char* color, double strokeWidth) {
   std::vector<std::pair<double, double> > stepped;

   stepped.reserve(points.size() * 2);

   for (std::size_t i = 0; i < points.size(); ++i) {
      if (i > 0) {
         stepped.emplace_back(points[i].first, points[i - 1].second); // значение держится до перехода
      }
      stepped.push_back(points[i]);
   }
   return polyline(stepped, color, strokeWidth);
}

SvgCanvas &SvgCanvas::envelope(const std::vector<Bin>& bins, const char* color, double opacity) {
   if (bins.empty()) {
      return *this;
   }
   std::string element = "<polygon points=\"";

   for (const Bin& bin : bins) {
      element += format("%.1f", fx(bin.center)) + "," + format("%.1f", fy(bin.high)) + " ";
   }

   for (auto it = bins.rbegin(); it != bins.rend(); ++it) {
      element += format("%.1f", fx(it->center)) + "," + format("%.1f", fy(it->low)) + " ";
   }
   element += "\" fill=\"";
   element += color;
   element += "\" fill-opacity=" + format("\"%.3f\"", opacity) + "/>";
   body_.push_back(std::move(element));
   return *this;
}

SvgCanvas &SvgCanvas::bars(const std::vector<double>& edges, const std::vector<double>& values,
                           const char* color, double opacity) {
   const double base = std::max(yLo_, 0.0);

   for (std::size_t i = 0; (i < values.size()) && ((i + 1) < edges.size()); ++i) {
      const double x0 = fx(edges[i]);
      const double x1 = fx(edges[i + 1]);
      const double y0 = fy(std::max(values[i], base));
      const double y1 = fy(base);

      if ((y1 - y0) <= 0.05) {
         continue;
      }
      char element[256];

      std::snprintf(element, sizeof(element),
                    "<rect x=\"%.2f\" y=\"%.2f\" width=\"%.2f\" height=\"%.2f\" fill=\"%s\" "
                    "fill-opacity=\"%.3f\"/>", x0, y0, std::max(x1 - x0 - 0.4, 0.4), y1 - y0,
                    color, opacity);
      body_.emplace_back(element);
   }
   return *this;
}

SvgCanvas &SvgCanvas::rectangle(double x0, double x1, double y0, double y1, const char* color,
                                double opacity, bool stroke) {
   char element[320];

   std::snprintf(element, sizeof(element),
                 "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"%s\" "
                 "fill-opacity=\"%.3f\"%s%s%s/>", fx(x0), fy(y1), fx(x1) - fx(x0),
                 fy(y0) - fy(y1), color, opacity, stroke ? " stroke=\"" : "",
                 stroke ? color : "", stroke ? "\" stroke-width=\"1\"" : "");
   body_.emplace_back(element);
   return *this;
}

SvgCanvas &SvgCanvas::labelAt(double x, double y, const std::string& text, int size,
                              const char* color, const char* anchor) {
   overlay_.push_back(textElement(fx(x), fy(y), text, size, color, "400", anchor));
   return *this;
}

SvgCanvas &SvgCanvas::note(double xPixels, double yPixels, const std::string& text, int size,
                           const char* color, const char* anchor) {
   overlay_.push_back(textElement(xPixels, yPixels, text, size, color, "400", anchor));
   return *this;
}

SvgCanvas &SvgCanvas::legend(std::vector<LegendRow> rows, LegendCorner corner) {
   legendRows_   = std::move(rows);
   legendCorner_ = corner;
   return *this;
}

void SvgCanvas::appendGrid(std::string& out) const {
   struct Axis {
      bool horizontal;
      double lo, hi, step;
      int minor, digits;
   };
   const Axis axes[2] = { { true, xLo_, xHi_, xStep_, xMinor_, xDigits_ },
                          { false, yLo_, yHi_, yStep_, yMinor_, yDigits_ } };

   for (const Axis& axis : axes) {
      const double first = std::ceil(axis.lo / axis.step - 1e-9) * axis.step;

      // минорные деления
      for (double v = first - axis.step; v <= axis.hi + 1e-9; v += axis.step / axis.minor) {
         if ((v < axis.lo - 1e-9)
             || (std::fabs(v / axis.step - std::round(v / axis.step)) < 1e-9)) {
            continue;
         }
         char element[192];

         if (axis.horizontal) {
            std::snprintf(element, sizeof(element),
                          "<line x1=\"%.1f\" y1=\"%d\" x2=\"%.1f\" y2=\"%d\" stroke=\"%s\" "
                          "stroke-width=\"1\"/>", fx(v), svg::plotTop, fx(v), svg::plotBottom,
                          svg::gridMinor);
         } else {
            std::snprintf(element, sizeof(element),
                          "<line x1=\"%d\" y1=\"%.1f\" x2=\"%d\" y2=\"%.1f\" stroke=\"%s\" "
                          "stroke-width=\"1\"/>", svg::plotLeft, fy(v), svg::plotRight, fy(v),
                          svg::gridMinor);
         }
         out += element;
         out += '\n';
      }

      // мажорные деления с подписями
      for (double v = first; v <= axis.hi + 1e-9; v += axis.step) {
         char element[192];

         if (axis.horizontal) {
            std::snprintf(element, sizeof(element),
                          "<line x1=\"%.1f\" y1=\"%d\" x2=\"%.1f\" y2=\"%d\" stroke=\"%s\" "
                          "stroke-width=\"1\"/>", fx(v), svg::plotTop, fx(v), svg::plotBottom,
                          svg::gridMajor);
            out += element;
            out += '\n';
            out += textElement(fx(v), svg::plotBottom + 20, numberRu(v, axis.digits), 11,
                               svg::textSecond, "400", "middle");
         } else {
            std::snprintf(element, sizeof(element),
                          "<line x1=\"%d\" y1=\"%.1f\" x2=\"%d\" y2=\"%.1f\" stroke=\"%s\" "
                          "stroke-width=\"1\"/>", svg::plotLeft, fy(v), svg::plotRight, fy(v),
                          svg::gridMajor);
            out += element;
            out += '\n';
            out += textElement(svg::plotLeft - 10, fy(v) + 4, numberRu(v, axis.digits), 11,
                               svg::textSecond, "400", "end");
         }
         out += '\n';
      }
   }
}

void SvgCanvas::appendLegend(std::string& out) const {
   if (legendRows_.empty()) {
      return;
   }
   std::size_t longest = 0;

   for (const LegendRow& row : legendRows_) {
      longest = std::max(longest, row.label.size());
   }
   // Оценка ширины: подпись в UTF-8, кириллица занимает два байта на знак
   const int box    = 16 + 46 + static_cast<int> (longest * 3.6);
   const int rows   = static_cast<int> (legendRows_.size());
   const int boxH   = 12 + 18 * rows;
   const bool right = (legendCorner_ == LegendCorner::rightTop)
                      || (legendCorner_ == LegendCorner::rightBottom);
   const bool top = (legendCorner_ == LegendCorner::rightTop)
                    || (legendCorner_ == LegendCorner::leftTop);
   const double lx = right ? (svg::plotRight - box - 12) : (svg::plotLeft + 12);
   const double ly = top ? (svg::plotTop + 12) : (svg::plotBottom - boxH - 12);
   char frame[256];

   std::snprintf(frame, sizeof(frame),
                 "<rect x=\"%.1f\" y=\"%.1f\" width=\"%d\" height=\"%d\" fill=\"%s\" "
                 "fill-opacity=\"0.94\" stroke=\"%s\" stroke-width=\"1\"/>", lx, ly, box, boxH,
                 svg::background, svg::gridMajor);
   out += frame;
   out += '\n';

   for (int i = 0; i < rows; ++i) {
      const LegendRow& row = legendRows_[static_cast<std::size_t> (i)];
      const double y       = ly + 21 + 18 * i;
      char sample[256];

      if (std::string(row.dash) == svg::dashFill) {
         std::snprintf(sample, sizeof(sample),
                       "<rect x=\"%.1f\" y=\"%.1f\" width=\"28\" height=\"12\" fill=\"%s\" "
                       "fill-opacity=\"0.18\" stroke=\"%s\" stroke-opacity=\"0.5\"/>",
                       lx + 12, y - 11, row.color, row.color);
      } else {
         std::snprintf(sample, sizeof(sample),
                       "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"%s\" "
                       "stroke-width=\"1.8\" stroke-dasharray=\"%s\"/>", lx + 12, y - 4,
                       lx + 40, y - 4, row.color, row.dash);
      }
      out += sample;
      out += '\n';
      out += textElement(lx + 48, y, row.label, 12, svg::textPrimary);
      out += '\n';
   }
}

std::string SvgCanvas::str() const {
   std::string out;
   char header[256];

   std::snprintf(header, sizeof(header),
                 "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 %d %d\" width=\"%d\" "
                 "height=\"%d\">\n", svg::width, svg::height, svg::width, svg::height);
   out += header;
   out += "<title>" + escapeXml(title_) + "</title>\n";
   out += "<desc>" + escapeXml(description_) + "</desc>\n";
   std::snprintf(header, sizeof(header),
                 "<defs><clipPath id=\"plot\"><rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\"/>"
                 "</clipPath></defs>\n", svg::plotLeft, svg::plotTop,
                 svg::plotRight - svg::plotLeft, svg::plotBottom - svg::plotTop);
   out += header;
   std::snprintf(header, sizeof(header),
                 "<rect x=\"0\" y=\"0\" width=\"%d\" height=\"%d\" fill=\"%s\"/>\n", svg::width,
                 svg::height, svg::background);
   out += header;
   out += textElement(svg::marginLeft, 34, title_, 18, svg::textPrimary, "600");
   out += '\n';

   for (std::size_t i = 0; (i < subtitles_.size()) && (i < 2); ++i) {
      out += textElement(svg::marginLeft, 54 + 16 * static_cast<double> (i), subtitles_[i], 12,
                         svg::textSecond);
      out += '\n';
   }
   appendGrid(out);
   out += "<g clip-path=\"url(#plot)\">\n";

   for (const std::string& element : body_) {
      out += element;
      out += '\n';
   }
   out += "</g>\n";

   for (const std::string& element : overlay_) {
      out += element;
      out += '\n';
   }
   std::snprintf(header, sizeof(header),
                 "<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" fill=\"none\" stroke=\"%s\" "
                 "stroke-width=\"1.2\"/>\n", svg::plotLeft, svg::plotTop,
                 svg::plotRight - svg::plotLeft, svg::plotBottom - svg::plotTop, svg::axisColor);
   out += header;
   out += textElement((svg::plotLeft + svg::plotRight) / 2.0, svg::plotBottom + 46, xLabel_, 13,
                      svg::textPrimary, "400", "middle");
   out += '\n';
   const std::string transform = " transform=\"translate(20,"
                                 + format("%.1f", (svg::plotTop + svg::plotBottom) / 2.0)
                                 + ") rotate(-90)\"";

   out += textElement(0, 0, yLabel_, 13, svg::textPrimary, "400", "middle", transform);
   out += '\n';
   appendLegend(out);
   out += "</svg>";
   return out;
}
} // namespace glonass_service
