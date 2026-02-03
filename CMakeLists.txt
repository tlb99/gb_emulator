cmake_minimum_required(VERSION 4.1)
project(gb_emulator)

set(CMAKE_CXX_STANDARD 20)

find_package(SDL2 REQUIRED)

add_executable(gb_emulator main.cpp
        src/memory/game.cc
        src/processor/cpu.cc
        src/game_boy.cc
        src/memory/wram.cc
        src/graphics/ppu.cc
)

target_include_directories(gb_emulator PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(gb_emulator PRIVATE SDL2::SDL2)

# Tests
include(CTest)
find_package(Catch2 3 REQUIRED)
add_executable(gb_tests tests/test_game_boy.cc)
target_link_libraries(gb_tests PRIVATE Catch2::Catch2WithMain)
