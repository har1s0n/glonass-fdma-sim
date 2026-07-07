#include "glonass/ranging_code.h"
#include <cassert>
#include <cstddef>

namespace glonass {
Bit RangingCode::at(int chipIndex) const {
   assert(chipIndex >= 0 && chipIndex < codeLength); // А.6: 0 <= chipIndex <= 510
   return codeTable_[static_cast<std::size_t> (chipIndex)];
}

const std::array<Bit, codeLength> &RangingCode::table() const {
   return codeTable_;
}
} // namespace glonass
