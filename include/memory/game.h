//
// Created by Tony on 1/27/2026.
//

#ifndef GB_EMULATOR_GBGAME_H
#define GB_EMULATOR_GBGAME_H
#include <filesystem>
#include <vector>

namespace GameBoy {
  class Game {
  public:
    Game() = default;
    static bool loadFromFile(const std::string& game_path);

  private:
    static inline bool is_game_loaded_ = false;
    static inline std::vector<std::byte> memory_;
  };
}

#endif  // GB_EMULATOR_GBGAME_H
