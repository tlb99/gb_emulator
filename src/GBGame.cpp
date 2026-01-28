//
// Created by Tony on 1/27/2026.
//

#include "../include/GBGame.h"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <system_error>

bool GBGame::loadFromFile(const std::string& game_path) {
  // Load game
  std::ifstream input_game(game_path, std::ios::binary);

  // Load game size using non-throwable overload
  std::error_code ec;
  const uintmax_t size = std::filesystem::file_size(game_path, ec);

  // Check if the provided game file can be opened and the size can be obtained
  if (input_game.is_open() && !ec) {
    memory_.resize(size);
    input_game.read(reinterpret_cast<char*>(memory_.data()), static_cast<std::streamsize>(size)); // Explicit cast; GB games are not bigger than long long, not an issue
    input_game.close();
    is_game_loaded_ = true;
    return true;
  }

  input_game.close();
  return false;
}