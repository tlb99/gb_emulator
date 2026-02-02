//
// Created by Tony on 1/30/2026.
//

#include "game_boy.h"

bool GameBoy::loadGame(const std::string& path) {
  return game_.loadFromFile(path);
}

void GameBoy::run() {

}