#include <fstream>
#include <iostream>

#include "include/memory/game.h"

// TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or click the <icon
// src="AllIcons.Actions.Execute"/> icon in the gutter.
int main(int argc, char* argv[]) {
  if (argc < 2) {
      std::cout << "no game provided, press any key to exit..." << std::endl;
      std::cin;
      exit(0);
  }

  GameBoy::Game Game;

  const bool loaded = GameBoy::Game::loadFromFile(argv[1]);

  return 0;
}