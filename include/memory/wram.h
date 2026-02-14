//
// Created by Tony on 1/30/2026.
//

#ifndef GB_EMULATOR_MEMORY_H
#define GB_EMULATOR_MEMORY_H
#include <cstdint>

#include "ram.h"

class WRAM : public RAM {
public:
  static constexpr uint16_t START = 0xC000;
  static constexpr uint16_t END = 0xDFFF;
  static constexpr uint16_t SIZE = 0x2000;  // According to Pan Docs, WRAM is 8 KiB by default on most Game Boy models but the Game Boy Color, which has 32 KiB

  WRAM() : RAM(START, END, SIZE) {}

protected:
  [[nodiscard]] std::string get_class_name() const override { return "WRAM";}
};

#endif //GB_EMULATOR_MEMORY_H