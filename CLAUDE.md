# Project Instructions for C++ Game Boy emulator

## 🚨 CRITICAL: Code Standards & Guidelines
1.  **Language Standard**: All C++ code must adhere to the C++20 standard.
2.  **Language Style**: C++ code must follow the Google C++ Style.
3.  **Naming Conventions**: Follow the Google C++ style (PascalCase for classes and functions, snake_case for file names, .cc file name extensions, etc.).
4.  **Error Handling**: Use exceptions for exceptional cases. Do not use raw pointers for ownership; prefer smart pointers (`std::unique_ptr`, `std::shared_ptr`).
5.  **Logging**: Use the `spdlog` library for all logging operations.
6.  **Testing**: All new functionality must eventually include corresponding unit tests using the `Catch2` framework in the `tests/` directory.

## Project Context
*   **Purpose**: This repository contains a work-in-progress implementation of a Nintendo Game Boy emulator.
*   **Dependencies**: `Catch2`, `SDL2`, `spdlog`,
*   **Architecture**: Modular design, prefer composition over inheritance.
