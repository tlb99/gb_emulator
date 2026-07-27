//
// Created by Tony on 2/23/2026.
//

#ifndef GB_EMULATOR_TILE_H
#define GB_EMULATOR_TILE_H
#include <cstdint>


class Tile {
public:

private:
  enum class Palette {
    WHITE,
    LIGHT_GRAY,
    DARK_GRAY,
    BLACK
  };

  Palette data_[8][8];
};


#endif //GB_EMULATOR_TILE_H