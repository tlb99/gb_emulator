//
// Created by Tony on 1/28/2026.
//

#ifndef GB_EMULATOR_CPU_H
#define GB_EMULATOR_CPU_H

#include <cstdint>
#include <functional>
#include <unordered_map>

#include "graphics/ppu.h"
#include "memory/game.h"
#include "memory/wram.h"
#include "memory/hram.h"



class CPU {
public:
  explicit CPU(WRAM& memory, HRAM& hram, Game& game, PPU& ppu)
  : game_(game), wram_(memory), hram_(hram), ppu_(ppu),
    a_(0x01), f_(0xB0), b_(0xFF), c_(0x13), d_(0x00), e_(0xC1), h_(0x84), l_(0x03),
    pc_(0x0100), sp_(0xFFFE), zero_(false), substraction_(false), half_carry_(false), carry_(false) {}

  void Cycle();

  // Register getters
  [[nodiscard]] const uint8_t& A() const { return a_; }
  [[nodiscard]] const uint8_t& F() const { return f_; }
  [[nodiscard]] const uint8_t& B() const { return b_; }
  [[nodiscard]] const uint8_t& C() const { return c_; }
  [[nodiscard]] const uint8_t& D() const { return d_; }
  [[nodiscard]] const uint8_t& E() const { return e_; }
  [[nodiscard]] const uint8_t& H() const { return h_; }
  [[nodiscard]] const uint8_t& L() const { return l_; }
  [[nodiscard]] const uint16_t& PC() const { return pc_; }
  [[nodiscard]] const uint16_t& SP() const { return sp_; }
  [[nodiscard]] const bool& ZERO() const { return zero_; }
  [[nodiscard]] const bool& SUBSTRACTION() const { return substraction_; }
  [[nodiscard]] const bool& HALF_CARRY() const { return half_carry_; }
  [[nodiscard]] const bool& CARRY() const { return carry_; }

private:
  enum class RegisterPair {
    AF,
    BC,
    DE,
    HL
  };

  const std::unordered_map<RegisterPair, std::pair<uint8_t&, uint8_t&>> register_pairs_ = {
    {RegisterPair::AF, {a_, f_}},
    {RegisterPair::BC, {b_, c_}},
    {RegisterPair::DE, {d_, e_}},
    {RegisterPair::HL, {h_, l_}}
  };

  const std::unordered_map<uint8_t, uint8_t&> special_arithmetic_registers_ = {
    {0, b_},
    {1, c_},
    {2, d_},
    {3, e_},
    {4, h_},
    {5, l_},
  };

  using ArithmeticFunction = std::function<void(uint8_t&, const uint8_t&)>;

  const std::vector<std::pair<std::pair<uint8_t, uint8_t>, ArithmeticFunction>> arithmetic_functions_ = {
    {{0x40, 0x45}, [this](uint8_t& left, const uint8_t& right) { ADDr8(left, right); }},
    {{0x46, 0x46}, [this](uint8_t& left, const uint8_t& right) { ADDr8(left, right); }},
    {{0x47, 0x87}, [this](uint8_t& left, const uint8_t& right) { ADDr8(left, right); }},
    {{0x80, 0x87}, [this](uint8_t& left, const uint8_t& right) { ADDr8(left, right); }},
    {{0x80, 0x87}, [this](uint8_t& left, const uint8_t& right) { ADDr8(left, right); }},
    {{0x80, 0x87}, [this](uint8_t& left, const uint8_t& right) { ADDr8(left, right); }},
    {{0x80, 0x87}, [this](uint8_t& left, const uint8_t& right) { ADDr8(left, right); }},
    {{0x88, 0x8E}, [this](uint8_t& left, const uint8_t& right) { ADCr8(left, right); }},
    {{0x90, 0x97}, [this](uint8_t& left, const uint8_t& right) { SUBr8(left, right); }},
    {{0x98, 0x9F}, [this](uint8_t& left, const uint8_t& right) { SBCr8(left, right); }},
    {{0xA0, 0xA7}, [this](uint8_t& left, const uint8_t& right) { ANDr8(left, right); }},
    {{0xA8, 0xAF}, [this](uint8_t& left, const uint8_t& right) { XORr8(left, right); }},
    {{0xB0, 0xB7}, [this](uint8_t& left, const uint8_t& right) { ORr8(left, right);  }},
    {{0xB8, 0xBF}, [this](const uint8_t& left, const uint8_t& right) { CPr8(left, right);  }},
};

  /* 1-byte opcodes */

  /**
 * @brief Writes the value of source_reg into the WRAM address obtained by combining high_reg and low_reg.
 *
 * This function writes the 8-bit value of source_reg into the 16-bit WRAM address
 * obtained by combining the 8-bit high_reg and low_reg registers using bit shifting
 * and the OR bitwise operator.
 *
 * @param pair
 * @param source
 */
  void LDr16r8(RegisterPair pair, const uint8_t& source) const;

  // Decrements 8-bit register
  void DECr8(uint8_t& reg);

  void LDr8n8(RegisterPair pair);
  static void LDr8r8(uint8_t& destination, const uint8_t& source);
  void LDHr8r8(const uint8_t &source, const uint8_t &offset);

  void CPr8(const uint8_t& left_reg, const uint8_t& right_reg);

  // 2-byte opcodes
  void LDn8(uint8_t& reg);
  void LDHa8r8(const uint8_t& source);
  void LDHr8a8(uint8_t &destination);

  void CPn8(const uint8_t& source);

  void JRe8(const bool &condition);

  // 3-byte opcodes
  void LDn16(RegisterPair pair);

  void LDr16n16(uint16_t &reg);
  void LDa16r8(const uint8_t& reg);
  void DECr16(RegisterPair pair) const;
  void INCr16(RegisterPair pair) const;

  /* ALU operations */

  void ADDr8(uint8_t& left_reg, const uint8_t& right_reg);
  void ADCr8(uint8_t& left_reg, const uint8_t& right_reg);

  void SUBr8(uint8_t& left_reg, const uint8_t& right_reg);
  void SBCr8(uint8_t& left_reg, const uint8_t& right_reg);

  void ANDr8(uint8_t& left_reg, const uint8_t& right_reg);

  void ORr8(uint8_t& left_reg, const uint8_t& right_reg);
  void XORr8(uint8_t& left_reg, const uint8_t& right_reg);

  // Helper function to combine two 8-bit registers into a 16-bit value
  [[nodiscard]] uint16_t CombineRegisters(RegisterPair pair) const;

  // Helper function to combine two bytes into a 16-bit value
  static uint16_t CombineBytes(const uint8_t& hi, const uint8_t& lo) ;

  static std::pair<uint8_t, uint8_t> SplitBytes(const uint16_t& value) ;

  Game& game_;
  WRAM& wram_;
  HRAM hram_;
  PPU& ppu_;

  // 8-bit Registers
  uint8_t a_;
  uint8_t f_;

  uint8_t b_;
  uint8_t c_;

  uint8_t d_;
  uint8_t e_;

  uint8_t h_;
  uint8_t l_;

  // Program counter and stack pointer
  uint16_t pc_;  // Program counter -- initialized to 0x0100, where the Game Boy first starts executing game instructions
  uint16_t sp_;  // Stack pointer

  // Flags
  bool zero_;
  bool substraction_;
  bool half_carry_;
  bool carry_;

};

#endif  // GB_EMULATOR_CPU_H
