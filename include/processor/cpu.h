//
// Created by Tony on 1/28/2026.
//

#ifndef GB_EMULATOR_CPU_H
#define GB_EMULATOR_CPU_H

namespace GameBoy {

class CPU {
  // TODO structure this class to match the Game Boy's CPU
public:
  void Fetch();
  void Decode();
  void Execute();
private:

};

}  // namespace GameBoy

#endif  // GB_EMULATOR_CPU_H
