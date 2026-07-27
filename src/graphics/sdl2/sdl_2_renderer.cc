//
// Created by Tony on 2/23/2026.
//

#include "graphics/sdl2/sdl_2_renderer.h"

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

      //Fill the surface white
      SDL_FillRect( surface_, nullptr, SDL_MapRGB( surface_->format, 0xFF, 0xFF, 0xFF ) );

      //Update the surface
      SDL_UpdateWindowSurface( window_ );

      //Hack to get window to stay up
      SDL_Event e; bool quit = false; while( quit == false ){ while( SDL_PollEvent( &e ) ){ if( e.type == SDL_QUIT ) quit = true; } }
    }
  }

  return success;
}