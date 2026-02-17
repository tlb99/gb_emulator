//
// Created by Tony on 2/13/2026.
//

#ifndef GB_EMULATOR_HRAM_H
#define GB_EMULATOR_HRAM_H
#include <cstdint>
#include <string>

#include "ram.h"


class HRAM : public RAM {
public:
  static constexpr uint16_t START = 0xFF80;
  static constexpr uint16_t END = 0xFFFE;
  static constexpr uint16_t SIZE = 0xFE;

  HRAM() : RAM(START, END, SIZE) {}

protected:
  [[nodiscard]] std::string get_class_name() const override { return "HRAM";}
};

#endif //GB_EMULATOR_HRAM_H