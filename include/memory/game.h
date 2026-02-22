//
// Created by Tony on 1/27/2026.
//

#ifndef GB_EMULATOR_GBGAME_H
#define GB_EMULATOR_GBGAME_H
#include <filesystem>
#include <vector>

#include "memory.h"

class Game : public Memory {
public:
  static constexpr uint16_t START = 0x0000;
  static constexpr uint16_t END = 0x7FFF;
  static constexpr uint16_t SIZE = 0x8000;

  Game() : Memory(START, END, SIZE), is_game_loaded_(false), rom_(0) {};
  bool LoadFromFile(const std::string& game_path);
  [[nodiscard]] bool IsGameLoaded() const { return is_game_loaded_; }
  [[nodiscard]] const std::vector<uint8_t>& ROM() const { return rom_; }
  void LoadFromBuffer(const std::vector<uint8_t>& buffer) { rom_ = buffer; is_game_loaded_ = true; }

protected:
  [[nodiscard]] std::string get_class_name() const override { return "ROM";}

private:
  bool is_game_loaded_;
  std::vector<uint8_t> rom_;
};

#endif  // GB_EMULATOR_GBGAME_H
