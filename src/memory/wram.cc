//
// Created by Tony on 1/30/2026.
//

#include <spdlog/spdlog.h>
#include "memory/wram.h"

void WRAM::Write(const uint8_t& value, const uint16_t& address) {
  if (address >= WRAM_START && address <= WRAM_END) {
    wram_[address - WRAM_START] = value;
  } else {
    spdlog::warn("Invalid WRAM write: addr=0x{:04X}, value=0x{:02X}", address, value);
  }
}

uint8_t WRAM::Read(const uint16_t& address) const {
  if (address >= WRAM_START && address <= WRAM_END) {
    return wram_[address - WRAM_START];
  }

  spdlog::warn("Invalid WRAM read: addr=0x{:04X}", address);
  return 0xFF;
}