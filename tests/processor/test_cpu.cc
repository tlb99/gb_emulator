//
// Created by Tony on 2/4/2026.
//
#include <catch2/catch_test_macros.hpp>

#include "processor/cpu.h"
#include "memory/rom.h"
#include "memory/wram.h"
#include "graphics/ppu.h"

TEST_CASE( "CPU opcode testing", "[cpu]" ) {
  // Test setup for all sections
  ROM game;
  WRAM wram;
  HRAM hram;
  PPU ppu;

  CPU cpu(wram,hram, game, ppu);

  SECTION("Opcode 0x05 (DECr8)") {
    SECTION("DECr8 sets zero flag when decrementing register to zero") {
      std::vector<uint8_t> rom(0x0103, 0x00);
      rom[0x0100] = 0x06; // LD B, n8
      rom[0x0101] = 0x01; // n8 = 1
      rom[0x0102] = 0x05; // DEC B
      game.LoadFromBuffer(rom);

      cpu.Cycle(); // Execute LD B, 1
      REQUIRE(cpu.B() == 1);
      REQUIRE(cpu.ZERO() == false);

      cpu.Cycle(); // Execute DEC B
      REQUIRE(cpu.B() == 0);
      REQUIRE(cpu.ZERO() == true);
    }

    SECTION("DECr8 sets subtraction flag to true") {
      std::vector<uint8_t> rom(0x0103, 0x00);
      rom[0x0100] = 0x06; // LD B, 1
      rom[0x0101] = 0x01;
      rom[0x0102] = 0x05; // DEC B
      game.LoadFromBuffer(rom);

      cpu.Cycle(); // LD B, 1
      cpu.Cycle(); // DEC B

      REQUIRE(cpu.SUBSTRACTION() == true);
    }

    SECTION("DECr8 sets half carry flag") {
      std::vector<uint8_t> rom(0x0103, 0x00);
      rom[0x0100] = 0x06; // LD B, 0x10
      rom[0x0101] = 0x10;
      rom[0x0102] = 0x05; // DEC B
      game.LoadFromBuffer(rom);

      cpu.Cycle(); // LD B, 0x10
      cpu.Cycle(); // DEC B

      REQUIRE(cpu.HALF_CARRY() == true);
    }

    SECTION("DECr8 sets carry flag on underflow") {
      std::vector<uint8_t> rom(0x0103, 0x00);
      rom[0x0100] = 0x06; // LD B, 0
      rom[0x0101] = 0x00;
      rom[0x0102] = 0x05; // DEC B
      game.LoadFromBuffer(rom);

      cpu.Cycle(); // LD B, 0
      cpu.Cycle(); // DEC B

      REQUIRE(cpu.B() == 0xFF);
      REQUIRE(cpu.CARRY() == true);
    }
  }
}
