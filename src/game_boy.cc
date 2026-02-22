//
// Created by Tony on 1/30/2026.
//

#include "game_boy.h"

bool GameBoy::loadGame(const std::string& path) {
  return rom_.LoadFromFile(path);
}

void GameBoy::run() {
  if (!rom_.IsGameLoaded())
    return;

  while (true)
    cpu_.Cycle();
}