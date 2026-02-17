//
// Created by Tony on 1/28/2026.
//

#include "processor/cpu.h"



#include <spdlog/spdlog.h>

// TODO implement memory bus to determine which address corresponds to which component
void CPU::Cycle() {
  // Fetch
  const uint8_t opcode = game_.ROM()[pc_++];

  spdlog::info( std::format("Processing opcode {:#04X} at ROM address {:#04X}", static_cast<unsigned int>(opcode),
    static_cast<unsigned int>(pc_ - 1)));

  // Check if opcode falls within special ranges
  for (const auto& [ranges, func] : arithmetic_functions_) {
    if (auto [start, end] = ranges; !(start <= opcode && opcode <= end)) continue;

    const uint8_t last_digit = opcode & 0xF;
    const uint8_t last_digit_mod = last_digit > 0x7 ? last_digit - 0x8 : last_digit;
    auto it = left_register_ranges_.lower_bound(opcode);

    // Exact opcode not found
    if (it == left_register_ranges_.end()) {
      --it;
    }

    uint8_t& left_register = it->second;

    if (last_digit == 0x6 || last_digit == 0xE)
      func(left_register, wram_.Read(CombineRegisters(RegisterPair::HL)));
    else if (last_digit == 0x7 || last_digit == 0xF)
      func(left_register, a_);
    else
      func(left_register, register_ranges_.at(last_digit_mod));

    return;
  }

  // Decode & Execute
  switch (opcode) {
    // NOP -- Do nothing
    case 0x00:
      break;
    // LD BC, n16
    case 0x1:
      LDn16(RegisterPair::BC);
      break;
    // INC B
    case 0x04:
      INCr8(b_);
      break;
    // DEC B
    case 0x05:
      DECr8(b_);
      break;
    // LD B, n8
    case 0x06:
      LDn8(b_);
      break;
    // DEC BC
    case 0x0B:
      DECr16(RegisterPair::BC);
      break;
    // INC C
    case 0x0C:
      INCr8(c_);
      break;
    // DEC C
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
    // LD A, [HL+]
    case 0x2A:
      LDr16r8( RegisterPair::HL, a_);
      INCr16(RegisterPair::HL);
      break;
    // CPL
    case 0x2F:
      CPL();
      break;
      // LD SP, n16
    case 0x31:
      LDr16n16(sp_);
      break;
    // LD [HL-], A
    case 0x32:
      LDr16r8(RegisterPair::HL, a_);
      DECr16(RegisterPair::HL);
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
    // CALL a16
    case 0xCD: {
      const uint8_t lo = game_.ROM()[pc_];
      const uint8_t hi = game_.ROM()[pc_ + 1];
      pc_ += 2;

      auto [pc_hi, pc_lo] = SplitBytes(pc_);
      memory_bus_write_(pc_hi, sp_);
      memory_bus_write_(pc_lo, sp_ - 1);
      sp_ -= 2;

      pc_ = CombineBytes(hi, lo);
      break;
    }// RET
    case 0xC9:
      RET();
      break;
    // LDH [a8], A
    case 0xE0:
      LDHn16r8(a_);
      break;
    // LDH [C], A
    case 0xE2:
      LDHr8r8(a_, c_);
      break;
    // AND A, n8
    case 0xE6:
      ANDr8n8(a_);
      break;
      // LD [a16], A
    case 0xEA:
      LDa16r8(a_);
      break;
    // LDH A, [a8]
    case 0xF0:
      LDHr8a8(a_);
      break;
    // EI -- TODO
    case 0xFB:
      // DI -- TODO once input is implemented
    case 0xF3:
      break;
    // CP A, n8
    case 0xFE:
      CPn8(a_);
      break;
    default:
      spdlog::error( std::format("Unimplemented opcode: {:#X} at address {:#X}", static_cast<unsigned int>(opcode), pc_ - 1));
      exit(0);
  }

}

void CPU::LDr16r8(const RegisterPair pair, const uint8_t &source) const {
  wram_.Write(source, CombineRegisters(pair));
}

void CPU::DECr8(uint8_t& reg) {
  substraction_ = true;
  half_carry_ = (reg & 0x10) == 1;
  --reg;
  zero_ = reg == 0;
}

void CPU::LDa16r8(const uint8_t& reg) {
  const uint8_t lo = game_.ROM()[pc_];
  const uint8_t hi = game_.ROM()[pc_ + 1];

  const uint16_t address = CombineBytes(hi, lo);
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

void CPU::LDHr8r8(const uint8_t &source, const uint8_t &offset) {
  wram_.Write(source, 0xFF00 + offset);
}

void CPU::CPr8(const uint8_t& left_reg, const uint8_t& right_reg) {
  uint8_t left_reg_copy = left_reg;
  SUBr8(left_reg_copy, right_reg);
}

void CPU::INCr8(uint8_t &reg) {
  ++reg;
  substraction_ = false;
  half_carry_ = (reg & 0xF) == 0;
  zero_ = reg == 0;
}

void CPU::RET() {
  const uint8_t& lo = memory_bus_read_(sp_ + 1);
  const uint8_t& hi = memory_bus_read_(sp_ + 2);
  sp_ += 2;
  pc_ = CombineBytes(hi, lo);
}

void CPU::CPL() {
  a_ = ~a_;
  substraction_ = false;
  half_carry_ = true;
}

void CPU::LDn8(uint8_t& reg) {
  const uint8_t byte = game_.ROM()[pc_];
  reg = byte;
  ++pc_;
}

void CPU::LDHn16r8(const uint8_t& source) {
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
  pc_ += static_cast<int8_t>(game_.ROM()[pc_]) + 1;
}

void CPU::ANDr8n8(uint8_t& reg) {
  const uint8_t& byte = game_.ROM()[pc_++];
  ANDr8(reg, byte);
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


void CPU::DECr16(const RegisterPair pair) const {
  // If decrementing lo leads to a decrease in hi, handle that, else return with a simple decrement to lo
  if (auto [hi, lo] = register_pairs_.at(pair); lo == 0) {
    --hi;
    lo = 0xFF;
  } else {
    --lo;
  }
}

void CPU::INCr16(const RegisterPair pair) const {
  if (auto [hi, lo] = register_pairs_.at(pair); lo == 0xFF) {
    ++hi;
    lo = 0;
  } else {
    ++lo;
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

RAM* CPU::get_memory_bus_(const uint16_t& address) const {
  for (const auto& range : memory_bus_map_) {
    if (range.start <= address && address <= range.end) {
      return range.ram_ptr;
    }
  }
  return nullptr;
}

void CPU::memory_bus_write_(const uint8_t& value, const uint16_t& address) const {
  RAM* ram = get_memory_bus_(address);

  ram->Write(value, address);
}

uint8_t CPU::memory_bus_read_(const uint16_t &address) const {
  const RAM* ram = get_memory_bus_(address);

  return ram->Read(address);
}

uint16_t CPU::CombineRegisters(const RegisterPair pair) const {
  auto [hi, lo] = register_pairs_.at(pair);
  return CombineBytes(hi, lo);
}

uint16_t CPU::CombineBytes(const uint8_t& hi, const uint8_t& lo) {
  return static_cast<uint16_t>(hi) << 8 | lo;
}

std::pair<uint8_t, uint8_t> CPU::SplitBytes(const uint16_t& value) {
  return {(value & 0xFF00) >> 8, value & 0xFF};
}