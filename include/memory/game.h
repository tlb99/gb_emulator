//
// Created by Tony on 1/27/2026.
//

#ifndef GB_EMULATOR_GBGAME_H
#define GB_EMULATOR_GBGAME_H
#include <filesystem>
#include <vector>

class Game {
public:
  Game() : is_game_loaded_(false), memory_(0) {};
  bool LoadFromFile(const std::string& game_path);
  [[nodiscard]] bool IsGameLoaded() const { return is_game_loaded_; }
  [[nodiscard]] const std::vector<uint8_t>& GetMemory() const { return memory_; }

private:
  bool is_game_loaded_;
  std::vector<uint8_t> memory_;
};

#endif  // GB_EMULATOR_GBGAME_H
