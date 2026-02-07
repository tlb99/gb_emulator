//
// Created by Tony on 1/28/2026.
//

#include "processor/cpu.h"

#include <iostream>

void CPU::Cycle() {
  // Fetch
  const uint8_t opcode = game_.ROM()[pc_++];

  std::cout << "Processing opcode 0x" << std::hex << static_cast<unsigned int>(opcode) <<
    " at ROM address 0x" << static_cast<unsigned int>(pc_ - 1) << std::endl;

  // Decode & Execute
  switch (opcode) {
    // NOP -- Do nothing
    case 0x00:
      break;
    // DEC B
    case 0x05:
      DECr8(b_);
      break;
      // LD B, n8
    case 0x06:
      LDn8(b_);
      break;
    case 0x0D:
      DECr8(c_);
      break;
    // LD C, n8
    case 0x0E:
      LDn8(c_);
      break;
    // JR NZ, e8
    case 0x20:
      JRe8(!zero_);
      break;
    // LD HL, n16 -- Load the next two little-endian bytes into registers H and L
    case 0x21:
      LDn16(RegisterPair::HL);
      break;
    // LD [HL-], A
    case 0x32:
      LDr16r8(RegisterPair::HL, a_);
      Dr16(RegisterPair::HL);
      break;
    // LD A, n8
    case 0x3E:
      LDn8(a_);
      break;
    // XOR A, A -- XOR register A with itself
    case 0xAF:
      a_ ^= a_;
      break;
    // JP a16 -- Jump to little-endian 16-bit address
    case 0xC3: {
      // Get the next two bytes, then set PC to the combo of these two bytes by bit shifting to the left
      const uint8_t lo = game_.ROM()[pc_];
      const uint8_t hi = game_.ROM()[pc_ + 1];
      pc_ = CombineBytes(hi, lo);
      break;
    }
    // LDH [a8], A
    case 0xE0:
      LDHa8(a_);
      break;
    // DI -- TODO once input is implemented
    case 0xF3:
      break;
    // RST $38
    case 0xFF: {
      break;
    }
    default:
      std::cout << "Unimplemented opcode: 0x" << std::hex << static_cast<
        unsigned int>(opcode) << std::endl;
      exit(0);
  }

}

void CPU::LDr16r8(const RegisterPair pair, const uint8_t &source) const {
  wram_.Write(source, CombineRegisters(pair));
}

void CPU::DECr8(uint8_t& reg) {
  SUBr8(reg, 1);
}

void CPU::LDn8(uint8_t& reg) {
  const uint8_t byte = game_.ROM()[pc_];
  reg = byte;
  ++pc_;
}

// TODO implement I/O range manipulation
void CPU::LDHa8(const uint8_t& source) {
  const uint8_t offset = game_.ROM()[pc_];
  //wram_.Write(source, 0xFF00 + offset);
  ++pc_;
}

void CPU::JRe8(const bool& condition) {
  if (!condition) {
    ++pc_;
    return;
  }
  pc_ += static_cast<int8_t>(game_.ROM()[pc_]);
}

void CPU::LDn16(const RegisterPair pair) {
  auto [hi, lo] = register_pairs_.at(pair);

  const uint8_t byte1 = game_.ROM()[pc_];
  const uint8_t byte2 = game_.ROM()[pc_ + 1];

  hi = byte2;
  lo = byte1;

  pc_+=2;
}

void CPU::Dr16(const RegisterPair pair) const {
  // If decrementing lo leads to a decrease in hi, handle that, else return with a simple decrement to lo
  if (auto [hi, lo] = register_pairs_.at(pair); lo == 0) {
    // Throw exception if hi and lo are both 0
    if (hi == 0)
      throw std::runtime_error("Attempted to decrement register below 0");

    --hi;
    lo = 0xFF;
  } else {
    --lo;
  }
}

void CPU::SUBr8(uint8_t& left_reg, const uint8_t& right_reg) {
  substraction_ = true;

  // Handle underflow case
  if (left_reg < right_reg) {
    half_carry_ = true;
    left_reg = 0xFF + (left_reg - right_reg) + 1;
  } else {
    left_reg -= right_reg;
  }

  zero_ = left_reg == 0;
}

uint16_t CPU::CombineRegisters(const RegisterPair pair) const {
  auto [hi, lo] = register_pairs_.at(pair);
  return CombineBytes(hi, lo);
}

uint16_t CPU::CombineBytes(const uint8_t& hi, const uint8_t& lo) {
  return static_cast<uint16_t>(hi) << 8 | lo;
}