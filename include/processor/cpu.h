//
// Created by Tony on 1/28/2026.
//

#ifndef GB_EMULATOR_CPU_H
#define GB_EMULATOR_CPU_H

#include <cstdint>

#include "memory/wram.h"

class CPU {
public:
  explicit CPU(WRAM& memory)
  : wram_(memory),
    a_(0), f_(0), b_(0), c_(0), d_(0), e_(0), h_(0), l_(0),
    pc_(0x0150), sp_(0), zero_(false), substraction_(false), half_carry_(false), carry_(false) {}
  void Cycle();

private:
  void Fetch();
  void Decode();
  void Execute();

  WRAM& wram_;

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
  uint16_t pc_;  // Program counter
  uint16_t sp_;  // Stack pointer

  // Flags
  bool zero_;
  bool substraction_;
  bool half_carry_;
  bool carry_;

};

#endif  // GB_EMULATOR_CPU_H
