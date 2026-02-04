//
// Created by Tony on 1/30/2026.
//

#include "memory/wram.h"

void WRAM::Write(const uint8_t value, const uint16_t address) {
  if (address >= 0xC000 && address < 0xE000) {
    wram_[address - 0xC000] = value;
  }
}