//
// Created by Tony on 1/30/2026.
//

#ifndef GB_EMULATOR_GAME_BOY_H
#define GB_EMULATOR_GAME_BOY_H
#include "graphics/ppu.h"
#include "memory/rom.h"
#include "processor/cpu.h"

class GameBoy {
public:
  GameBoy() : cpu_(memory_, hram_, vram_, rom_, ppu_) {}
  void run();
  bool loadGame(const std::string& path);

private:
  CPU cpu_;
  PPU ppu_;
  WRAM memory_;
  HRAM hram_;
  VRAM vram_;
  ROM rom_;
};

#endif //GB_EMULATOR_GAME_BOY_H