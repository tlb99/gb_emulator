//
// Created by Tony on 2/13/2026.
//

#include "memory/ram.h"

#include <spdlog/spdlog.h>

void RAM::Write(const uint8_t& value, const uint16_t& address) {
  if (address >= START && address <= END) {
    ram_[address - START] = value;
  } else {
    spdlog::warn("Invalid {} write: addr=0x{:04X}, value=0x{:02X}", get_class_name(), address, value);
  }
}

uint8_t RAM::Read(const uint16_t& address) const {
  if (address >= START && address <= END) {
    return ram_[address - START];
  }

  spdlog::warn("Invalid {} read: addr=0x{:04X}", get_class_name(), address);
  return 0xFF;
}
