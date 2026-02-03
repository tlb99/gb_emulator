//
// Created by Tony on 1/30/2026.
//

#ifndef GB_EMULATOR_MEMORY_H
#define GB_EMULATOR_MEMORY_H
#include <cstdint>
#include <vector>

class WRAM {
public:
  WRAM() : wram_(8192) {}
private:
  std::vector<uint8_t> wram_; // According to Pan Docs, WRAM is 8 KiB by default on most Game Boy models but the Game Boy Color, which has 32 KiB
};

#endif //GB_EMULATOR_MEMORY_H