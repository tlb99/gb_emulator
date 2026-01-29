cmake_minimum_required(VERSION 4.1)
project(gb_emulator)

set(CMAKE_CXX_STANDARD 20)

#find_package(SDL3 REQUIRED)

# Link the executable with SDL3 library
#target_link_libraries(gb_emulator PRIVATE SDL3::SDL3)

add_executable(gb_emulator main.cpp
        include/opcodes.h
        include/GBMemory.h
        include/GBGame.h
        src/GBGame.cpp
        src/cpu.cc
        src/cpu.h)
