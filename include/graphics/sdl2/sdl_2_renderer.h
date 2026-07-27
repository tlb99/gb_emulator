//
// Created by Tony on 2/23/2026.
//

#ifndef GB_EMULATOR_SDL_2_RENDERER_H
#define GB_EMULATOR_SDL_2_RENDERER_H
#include "SDL.h"


class SDL2Renderer {
public:
  // Hardcoded values for now
  const int kDefaultWidth = 160 * 5;
  const int kDefaultHeight = 144 * 5;

  SDL2Renderer() : window_(nullptr), surface_(nullptr) {}

  bool Init();
private:

  SDL_Window* window_;

  SDL_Surface* surface_;
};


#endif //GB_EMULATOR_SDL_2_RENDERER_H