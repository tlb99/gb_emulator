//
// Created by Tony on 1/30/2026.
//

#include "game_boy.h"

bool GameBoy::loadGame(const std::string& path) {
  return game_.LoadFromFile(path);
}

void GameBoy::run() {
  if (!game_.IsGameLoaded())
    return;

  while (true)
    cpu_.Cycle();
}