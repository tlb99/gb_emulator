cmake_minimum_required(VERSION 4.1)
project(gb_emulator)

set(CMAKE_CXX_STANDARD 20)

find_package(SDL2 REQUIRED)
find_package(spdlog REQUIRED)

add_executable(gb_emulator main.cpp
        src/memory/game.cc
        src/processor/cpu.cc
        src/game_boy.cc
        src/memory/wram.cc
        src/graphics/ppu.cc
)

target_include_directories(gb_emulator PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(gb_emulator PRIVATE SDL2::SDL2)
target_link_libraries(gb_emulator PRIVATE spdlog::spdlog)

find_package(Catch2 3 REQUIRED)

add_executable(gb_tests tests/test_game_boy.cc
        tests/memory/test_wram.cc
        src/memory/game.cc
        src/processor/cpu.cc
        src/game_boy.cc
        src/memory/wram.cc
        src/graphics/ppu.cc
        tests/processor/test_cpu.cc
)

target_include_directories(gb_tests PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(gb_tests PRIVATE spdlog::spdlog)
target_link_libraries(gb_tests PRIVATE Catch2::Catch2WithMain)
