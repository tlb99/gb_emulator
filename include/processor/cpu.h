//
// Created by Tony on 1/28/2026.
//

#ifndef GB_EMULATOR_CPU_H
#define GB_EMULATOR_CPU_H

#include <cstdint>
#include <unordered_map>

#include "graphics/ppu.h"
#include "memory/game.h"
#include "memory/wram.h"



class CPU {
public:
  explicit CPU(WRAM& memory, Game& game, PPU& ppu)
  : game_(game), wram_(memory), ppu_(ppu),
    a_(0x01), f_(0xB0), b_(0xFF), c_(0x13), d_(0x00), e_(0xC1), h_(0x84), l_(0x03),
    pc_(0x0100), sp_(0xFFFE), zero_(false), substraction_(false), half_carry_(false), carry_(false) {}
  void Cycle();

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

  // 2-byte opcodes
  void LDn8(uint8_t& reg);
  void LDHa8r8(const uint8_t& source);
  void LDHr8a8(uint8_t &destination);

  void CPn8(const uint8_t& source);

  void JRe8(const bool &condition);

  // 3-byte opcodes
  void LDn16(RegisterPair pair);

  // Decrements 16-bit register
  void Dr16(RegisterPair pair) const;

  /* ALU operations */

  // Subtract two registers from one another
  void SUBr8(uint8_t& left_reg, const uint8_t& right_reg);

  // Helper function to combine two 8-bit registers into a 16-bit value
  [[nodiscard]] uint16_t CombineRegisters(RegisterPair pair) const;

  // Helper function to combine two bytes into a 16-bit value
  static uint16_t CombineBytes(const uint8_t& hi, const uint8_t& lo) ;

  Game& game_;
  WRAM& wram_;
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
