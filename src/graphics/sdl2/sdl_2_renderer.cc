//
// Created by Tony on 2/23/2026.
//

#include "../../../include/graphics/sdl2/sdl_2_renderer.h"

#include <spdlog/spdlog.h>

bool SDL2Renderer::Init() {
  //Initialization flag
  bool success = true;

  //Initialize SDL
  if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
  {
    spdlog::error( "SDL could not initialize! SDL_Error: {}", SDL_GetError() );
    success = false;
  }
  else
  {
    //Create a window
    window_ = SDL_CreateWindow( "GB Emulator", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, kDefaultWidth, kDefaultHeight, SDL_WINDOW_SHOWN );
    if( window_ == nullptr )
    {
      spdlog::error( "Window could not be created! SDL_Error: {}", SDL_GetError() );
      success = false;
    }
    else
    {
      surface_ = SDL_GetWindowSurface( window_ );
    }
  }

  return success;
}