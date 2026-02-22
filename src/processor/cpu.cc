//
// Created by Tony on 1/28/2026.
//

#include "processor/cpu.h"



#include <spdlog/spdlog.h>

void CPU::Cycle() {
  // Fetch
  const uint8_t opcode = game_.ROM()[pc_++];

  spdlog::info( std::format("Processing opcode {:#04X} at ROM address {:#04X}", static_cast<unsigned int>(opcode),
    static_cast<unsigned int>(pc_ - 1)));

  // Check if opcode is an arithmetic/LD instruction (0x40-0xBF range)
  if (opcode >= 0x40 && opcode <= 0xBF && opcode != 0x76) {
    ProcessArithmeticOp(opcode);
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
    // LC [BC], A
    case 0x2:
      LDr16r8(RegisterPair::BC, a_);
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
    // LD D, n8
    case 0x16:
      LDn8(d_);
      break;
    // ADD HL, DE
    case 0x19:
      ADDr16r16(RegisterPair::HL, RegisterPair::DE);
      break;
      // JR NZ, e8
    case 0x20:
      JRe8(!zero_);
      break;
    // LD HL, n16 -- Load the next two little-endian bytes into registers H and L
    case 0x21:
      LDn16(RegisterPair::HL);
      break;
    // INC HL
    case 0x23:
      INCr16(RegisterPair::HL);
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
    // JP a16
    case 0xC3: {
      JPa16();
      break;
    }
    // PREFIX
    case 0xCB:
      PREFIX(game_.ROM()[pc_++]);
      break;
    // CALL a16
    case 0xCD: {
      const uint8_t lo = game_.ROM()[pc_];
      const uint8_t hi = game_.ROM()[pc_ + 1];
      pc_ += 2;

      auto [pc_hi, pc_lo] = SplitBytes(pc_);
      memory_bus_write_(pc_hi, sp_ - 1);
      memory_bus_write_(pc_lo, sp_ - 2);
      sp_ -= 2;

      pc_ = CombineBytes(hi, lo);
      break;
    }
    // RET
    case 0xC9:
      RET();
      break;
    // PUSH DE
    case 0xD5:
      PUSH(RegisterPair::DE);
      break;
      // LDH [a8], A
    case 0xE0:
      LDHn16r8(a_);
      break;
    // POP HL
    case 0xE1:
      POP(RegisterPair::HL);
      break;
    // LDH [C], A
    case 0xE2:
      LDHr8r8(a_, c_);
      break;
    // AND A, n8
    case 0xE6:
      ANDr8n8(a_);
      break;
    // JP HL
    case 0xE9:
      JPr16(RegisterPair::HL);
      break;
      // LD [a16], A
    case 0xEA:
      LDa16r8(a_);
      break;
    // RST $28
    case 0xEF:
      RST(0x28);
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

void CPU::JPa16() {
  const uint8_t lo = game_.ROM()[pc_];
  const uint8_t hi = game_.ROM()[pc_ + 1];
  pc_ = CombineBytes(hi, lo);
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
  const uint8_t& lo = memory_bus_read_(sp_);
  const uint8_t& hi = memory_bus_read_(sp_ + 1);
  sp_ += 2;
  pc_ = CombineBytes(hi, lo);
}

void CPU::CPL() {
  a_ = ~a_;
  substraction_ = false;
  half_carry_ = true;
}

void CPU::RST(const uint8_t& address) {
  ++pc_;

  auto [pc_hi, pc_lo] = SplitBytes(pc_);
  memory_bus_write_(pc_hi, sp_);
  memory_bus_write_(pc_lo, sp_ - 1);
  --sp_;

  pc_ = CombineBytes(0x00, address);
}

void CPU::POP(RegisterPair pair) {
  auto [pair_hi, pair_lo] = register_pairs_.at(pair);

  const uint8_t& lo = memory_bus_read_(sp_);
  const uint8_t& hi = memory_bus_read_(sp_ + 1);
  sp_ += 2;

  pair_hi = hi;
  pair_lo = lo;
}

void CPU::PUSH(const RegisterPair pair) {
  auto [pair_hi, pair_lo] = register_pairs_.at(pair);

  --sp_;
  memory_bus_write_(pair_hi, sp_);
  --sp_;
  memory_bus_write_(pair_lo, sp_);
}

void CPU::JPr16(const RegisterPair pair) {
  pc_ = CombineRegisters(pair);
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

void CPU::RLCr8(uint8_t& reg) {
  const uint8_t msb = reg & 0x80;
  carry_ = msb == 1;

  reg = (reg << 1) + msb;
  zero_ = reg == 0;
  substraction_ = false;
  half_carry_ = false;
}

void CPU::RRCr8(uint8_t& reg) {
  const uint8_t lsb = reg & 0x01;
  carry_ = lsb == 1;

  reg = (reg >> 1) + lsb;
  zero_ = reg == 0;
  substraction_ = false;
  half_carry_ = false;
}

void CPU::RLCr16(uint8_t& reg) {

}

void CPU::SWAPr8(uint8_t& reg) {
  reg = (reg << 4) + (reg >> 4);

  zero_ = reg == 0;
  substraction_ = false;
  half_carry_ = false;
  carry_ = false;
}

void CPU::RLr8(uint8_t& reg) {
  const bool old_carry = carry_;
  carry_ = (reg & 0x80) != 0;

  reg = (reg << 1) | (old_carry ? 1 : 0);
  zero_ = reg == 0;
  substraction_ = false;
  half_carry_ = false;
}

void CPU::RRr8(uint8_t& reg) {
  const bool old_carry = carry_;
  carry_ = (reg & 0x01) != 0;

  reg = (reg >> 1) | (old_carry ? 0x80 : 0);
  zero_ = reg == 0;
  substraction_ = false;
  half_carry_ = false;
}

void CPU::SLAr8(uint8_t& reg) {
  carry_ = (reg & 0x80) != 0;

  reg <<= 1;
  zero_ = reg == 0;
  substraction_ = false;
  half_carry_ = false;
}

void CPU::SRAr8(uint8_t& reg) {
  carry_ = (reg & 0x01) != 0;

  reg = (reg >> 1) | (reg & 0x80);
  zero_ = reg == 0;
  substraction_ = false;
  half_carry_ = false;
}

void CPU::SRLr8(uint8_t& reg) {
  carry_ = (reg & 0x01) != 0;

  reg >>= 1;
  zero_ = reg == 0;
  substraction_ = false;
  half_carry_ = false;
}

void CPU::BITr8(const uint8_t& reg, const uint8_t& bit) {
  zero_ = (reg & (1 << bit)) == 0;
  substraction_ = false;
  half_carry_ = true;
}

void CPU::RESr8(uint8_t& reg, const uint8_t& bit) {
  reg &= ~(1 << bit);
}

void CPU::SETr8(uint8_t& reg, const uint8_t& bit) {
  reg |= (1 << bit);
}

void CPU::PREFIX(const uint8_t &opcode) const {
  auto it = prefix_functions_.upper_bound(opcode);

  if (it != prefix_functions_.begin()) {
    --it;
  }

  const auto& func = it->second;
  const uint8_t reg_index = opcode & 0x7;

  if (const auto* p_func_r8 = std::get_if<std::function<void(uint8_t&)>>(&func)) {
    if (reg_index == 6) {
      const uint16_t hl = CombineRegisters(RegisterPair::HL);
      uint8_t value = memory_bus_read_(hl);
      (*p_func_r8)(value);
      memory_bus_write_(value, hl);
    } else {
      (*p_func_r8)(register_ranges_.at(reg_index));
    }
  } else if (const auto* p_func_bit = std::get_if<std::function<void(uint8_t&, const uint8_t&)>>(&func)) {
    const uint8_t bit = (opcode >> 3) & 0x7;
    if (reg_index == 6) {
      const uint16_t hl = CombineRegisters(RegisterPair::HL);
      uint8_t value = memory_bus_read_(hl);
      (*p_func_bit)(value, bit);
      memory_bus_write_(value, hl);
    } else {
      (*p_func_bit)(register_ranges_.at(reg_index), bit);
    }
  }

}

void CPU::ProcessArithmeticOp(const uint8_t &opcode) const {
  // Find the appropriate function based on the opcode base (upper 4 bits + lower 3 bits of upper nibble)
  const uint8_t opcode_base = opcode & 0xF8;
  auto it = arithmetic_functions_.upper_bound(opcode_base);

  if (it != arithmetic_functions_.begin()) {
    --it;
  }

  const auto& func = it->second;

  // Get destination register based on opcode range
  auto dest_it = destination_register_map_.upper_bound(opcode_base);
  if (dest_it != destination_register_map_.begin()) {
    --dest_it;
  }
  uint8_t& dest_register = dest_it->second;

  // Get source operand based on lower 3 bits of opcode
  const uint8_t source_index = opcode & 0x7;

  const auto* p_func = std::get_if<std::function<void(uint8_t&, const uint8_t&)>>(&func);

  if (source_index == 6) {
    // Source is (HL) - memory at address HL
    (*p_func)(dest_register, memory_bus_read_(CombineRegisters(RegisterPair::HL)));
  } else {
    // Source is a register
    (*p_func)(dest_register, register_ranges_.at(source_index));
  }
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

void CPU::ADDr16r16(RegisterPair left, RegisterPair right) {
  substraction_ = false;

  auto& [left_hi, left_lo] = register_pairs_.at(left);

  const uint16_t left_r16 = CombineRegisters(left);
  const uint16_t right_r16 = CombineRegisters(right);

  carry_ = right_r16 > 0xFFFF - left_r16;
  half_carry_ = (right_r16 & 0xFFF) > 0xFFF - (left_r16 & 0xFFF) ;
  const uint16_t sum = left_r16 + right_r16;

  auto [hi, lo] = SplitBytes(sum);
  left_hi = hi;
  left_lo = lo;
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
  for (const auto& [start, end, ram_ptr] : memory_bus_map_) {
    if (start <= address && address <= end) {
      return ram_ptr;
    }
  }
  spdlog::warn("Returning nullptr for memory bus address {:#04X}", address);
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