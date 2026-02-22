//
// Created by Tony on 2/22/2026.
//

#ifndef GB_EMULATOR_VRAM_H
#define GB_EMULATOR_VRAM_H
#include "ram.h"


class VRAM : public RAM {
public:
  static constexpr uint16_t START = 0x8000;
  static constexpr uint16_t END = 0x9FFF;
  static constexpr uint16_t SIZE = 0x2000;

  VRAM() : RAM(START, END, SIZE) {}

protected:
  [[nodiscard]] std::string get_class_name() const override { return "VRAM";}
};


#endif //GB_EMULATOR_VRAM_H