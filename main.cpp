#include <fstream>
#include <iostream>

#include "include/GBGame.h"

// TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or click the <icon
// src="AllIcons.Actions.Execute"/> icon in the gutter.
int main(int argc, char* argv[]) {
  if (argc < 2) {
      std::cout << "no game provided, press any key to exit..." << std::endl;
      std::cin;
      exit(0);
  }

  GBGame game;

  const bool loaded = GBGame::loadFromFile(argv[1]);

  std::cout << loaded << std::endl;

  return 0;

}