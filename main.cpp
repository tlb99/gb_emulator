#include <fstream>
#include <iostream>

#include "game_boy.h"
#include "include/memory/rom.h"
#include "SDL.h"

int main(int argc, char* argv[]) {
  if (argc < 2) {
      std::cout << "no game provided" << std::endl;
      exit(0);
  }

  GameBoy game_boy;
  game_boy.loadGame(argv[1]);
  game_boy.run();

  return 0;
}