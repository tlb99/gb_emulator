//
// Created by Tony on 1/28/2026.
//

#include "processor/cpu.h"



#include <iostream>

// TODO implement memory bus to determine which address corresponds to which component
void CPU::Cycle() {
  // Fetch
  const uint8_t opcode = game_.ROM()[pc_++];

  std::cout << "Processing opcode 0x" << std::hex << static_cast<unsigned int>(opcode) <<
    " at ROM address 0x" << static_cast<unsigned int>(pc_ - 1) << std::endl;

  // Check if opcode falls within special ranges
  for (const auto& [ranges, func] : arithmetic_functions_) {
    if (auto [start, end] = ranges; !(start <= opcode && opcode <= end)) continue;

    const uint8_t last_digit = opcode & 0xF;
    const uint8_t last_digit_mod = last_digit > 0x7 ? last_digit - 0x8 : last_digit;

    if (last_digit == 0x6 || last_digit == 0xE)
      func(a_, wram_.Read(CombineRegisters(RegisterPair::HL)));
    else if (last_digit == 0x7 || last_digit == 0xF)
      func(a_, a_);
    else
      func(a_, special_arithmetic_registers_.at(last_digit_mod));

    return;
  }

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
    // LD SP, n16
    case 0x31:
      LDr16n16(sp_);
      break;
    // LD [HL-], A
    case 0x32:
      LDr16r8(RegisterPair::HL, a_);
      Dr16(RegisterPair::HL);
      break;
    // LD [HL], n8
    case 0x36:
      LDr8n8(RegisterPair::HL);
      break;
    // LD A, n8
    case 0x3E:
      LDn8(a_);
      break;
    // LD B, B
    case 0x40:
      LDr8r8(b_,b_);
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
      LDHa8r8(a_);
      break;
    // LD [a16], A
    case 0xEA:
      LDa16r8(a_);
      break;
    // LDH A, [a8]
    case 0xF0:
      LDHr8a8(a_);
      break;
    // DI -- TODO once input is implemented
    case 0xF3:
      break;
    // CP A, n8
    case 0xFE:
      CPn8(a_);
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

void CPU::LDa16r8(const uint8_t& reg) {
  const uint8_t byte1 = game_.ROM()[pc_];
  const uint8_t byte2 = game_.ROM()[pc_ + 1];

  const uint16_t address = CombineBytes(byte2, byte1);
  wram_.Write(reg, address);

  pc_ += 2;
}

void CPU::LDr8n8(RegisterPair pair) {
  const uint8_t byte = game_.ROM()[pc_++];

  wram_.Write(byte, CombineRegisters(pair));
}

void CPU::LDr8r8(uint8_t& destination, const uint8_t& source) {
  destination = source;
}

void CPU::CPr8(const uint8_t& left_reg, const uint8_t& right_reg) {
  uint8_t left_reg_copy = left_reg;
  SUBr8(left_reg_copy, right_reg);
}

void CPU::LDn8(uint8_t& reg) {
  const uint8_t byte = game_.ROM()[pc_];
  reg = byte;
  ++pc_;
}

void CPU::LDHa8r8(const uint8_t& source) {
  const uint8_t offset = game_.ROM()[pc_++];
  wram_.Write(source, 0xFF00 + offset);
}

// TODO once I/O is in too
void CPU::LDHr8a8(uint8_t& destination) {
  zero_ = true;
  destination = 0x94; // hardcoded value for now, since tetris seems to expect A to be this value
  ++pc_;
}

void CPU::CPn8(const uint8_t& source) {
  const uint8_t& right = game_.ROM()[pc_++];
  uint8_t source_copy = source;
  SUBr8(source_copy, right);
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

void CPU::LDr16n16(uint16_t &reg) {
  const uint8_t lo = game_.ROM()[pc_];
  const uint8_t hi = game_.ROM()[pc_ + 1];

  reg = CombineBytes(hi, lo);
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

void CPU::ADDr8(uint8_t& left_reg, const uint8_t& right_reg) {
  substraction_ = false;

  carry_ = right_reg > 0xFF - left_reg;
  half_carry_ = (left_reg & 0xF) + (right_reg & 0xF) > 0xF;

  left_reg += right_reg;
  zero_ = left_reg == 0;
}

void CPU::ADCr8(uint8_t& left_reg, const uint8_t& right_reg) {
  substraction_ = false;
  const uint8_t c = carry_ ? 1 : 0;

  carry_ = right_reg > 0xFF - left_reg - c;
  half_carry_ = (left_reg & 0xF) + (right_reg & 0xF) + c > 0xF;

  left_reg += right_reg + c;
  zero_ = left_reg == 0;
}

void CPU::SUBr8(uint8_t& left_reg, const uint8_t& right_reg) {
  substraction_ = true;

  // Handle underflow case
  if (left_reg < right_reg) {
    carry_ = true;
  } else {
    half_carry_ = (left_reg & 0xF) == 0;
  }

  left_reg -= right_reg;
  zero_ = left_reg == 0;
}

void CPU::SBCr8(uint8_t& left_reg, const uint8_t& right_reg) {
  substraction_ = true;
  const uint8_t c = carry_ ? 1 : 0;

  // Handle underflow case
  if (left_reg < right_reg + c) {
    carry_ = true;
  } else {
    half_carry_ = (left_reg & 0xF) == 0;
  }

  left_reg -= right_reg - c;
  zero_ = left_reg == 0;
}

void CPU::ANDr8(uint8_t& left_reg, const uint8_t& right_reg) {
  left_reg &= right_reg;

  zero_ = left_reg == 0;
  substraction_ = false;
  half_carry_ = true;
  carry_ = false;
}

void CPU::ORr8(uint8_t& left_reg, const uint8_t& right_reg) {
  left_reg |= right_reg;

  zero_ = left_reg == 0;
  substraction_ = false;
  half_carry_ = false;
  carry_ = false;
}

void CPU::XORr8(uint8_t& left_reg, const uint8_t& right_reg) {
  left_reg ^= right_reg;

  zero_ = left_reg == 0;
  substraction_ = false;
  half_carry_ = false;
  carry_ = false;
}

uint16_t CPU::CombineRegisters(const RegisterPair pair) const {
  auto [hi, lo] = register_pairs_.at(pair);
  return CombineBytes(hi, lo);
}

uint16_t CPU::CombineBytes(const uint8_t& hi, const uint8_t& lo) {
  return static_cast<uint16_t>(hi) << 8 | lo;
}