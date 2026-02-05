//
// Created by Tony on 2/3/2026.
//

#include <catch2/catch_test_macros.hpp>

#include "memory/wram.h"

TEST_CASE( "WRAM allocates empty vector", "[wram]" ) {
  const WRAM wram;
  std::vector<uint8_t> empty_wram(WRAM::WRAM_SIZE);

  REQUIRE(wram.GetWRAM() == empty_wram);
}

TEST_CASE( "WRAM.Write() writes value at starting Game Boy offset address", "[wram]" ) {
  WRAM wram;
  constexpr uint8_t value = 0xFF;
  constexpr uint16_t address = WRAM::WRAM_START;

  wram.Write(value, address);

  REQUIRE(wram.Read(address) == value);
  REQUIRE(wram.GetWRAM()[address - WRAM::WRAM_START] == value);
}

TEST_CASE( "WRAM.Write() writes value at ending Game Boy offset address", "[wram]" ) {
  WRAM wram;
  constexpr uint8_t value = 0xFF;
  constexpr uint16_t address = WRAM::WRAM_END;

  wram.Write(value, address);

  REQUIRE(wram.Read(address) == value);
  REQUIRE(wram.GetWRAM()[address - WRAM::WRAM_START] == value);
}

TEST_CASE( "WRAM.Write() does not write value below starting Game Boy offset address ", "[wram]" ) {
  WRAM wram;
  constexpr uint8_t value = 0xF0;
  constexpr uint16_t address = 0x0000;

  wram.Write(value, address);

  REQUIRE(wram.Read(address) != value);
  REQUIRE(wram.GetWRAM()[address] != value);
}

TEST_CASE( "WRAM.Write() does not write value above ending Game Boy offset address ", "[wram]" ) {
  WRAM wram;
  constexpr uint8_t value = 0xF0;
  constexpr uint16_t address = WRAM::WRAM_END + 1;

  wram.Write(value, address);

  REQUIRE(wram.Read(address) != value);
}
