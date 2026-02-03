//
// Created by Tony on 1/28/2026.
//

#include "processor/cpu.h"

#include <iostream>

void CPU::Cycle() {
  const std::vector<uint8_t> rom = game_.GetMemory();

  // Fetch
  const uint8_t opcode = rom[pc_++];

  std::cout << "Processing opcode 0x" << std::hex << static_cast<unsigned int>(opcode) << std::endl;

  // Decode & Execute
  switch (opcode) {
    // NOP -- Do nothing
    case 0x00:
      break;
    // JP a16 -- Jump to little-endian 16-bit address
    case 0xC3: {
      // Get the next two bytes, then set PC to the combo of these two bytes by bit shifting to the left
      const uint8_t addr1 = rom[pc_];
      const uint8_t addr2 = rom[pc_ + 1];
      pc_ = static_cast<uint16_t>(addr2) << 8 | addr1;
      break;
    }
    default:
      std::cout << "Unimplemented opcode: 0x" << std::hex << static_cast<unsigned int>(opcode) << std::endl;
      exit(0);
  }

}