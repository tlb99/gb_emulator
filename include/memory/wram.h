//
// Created by Tony on 1/30/2026.
//

#ifndef GB_EMULATOR_MEMORY_H
#define GB_EMULATOR_MEMORY_H
#include <cstdint>
#include <vector>

class WRAM {
public:
  static constexpr uint16_t WRAM_START = 0xC000;
  static constexpr uint16_t WRAM_END = 0xDFFF;
  static constexpr size_t WRAM_SIZE = 0x2000;  // According to Pan Docs, WRAM is 8 KiB by default on most Game Boy models but the Game Boy Color, which has 32 KiB

  WRAM() : wram_(WRAM_SIZE) {}

  void Write(const uint8_t &value, const uint16_t &address);
  [[nodiscard]] uint8_t Read(const uint16_t &address) const;

  [[nodiscard]] const std::vector<uint8_t>& GetWRAM() const { return wram_; }
private:
  std::vector<uint8_t> wram_;
};

#endif //GB_EMULATOR_MEMORY_H