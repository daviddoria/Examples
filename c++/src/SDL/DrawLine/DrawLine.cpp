#include <SDL/SDL.h>
#include <SDL/SDL_draw.h>

#include <string>
#include <cassert>
#include <iostream>

#include <GL/glut.h>

SDL_Surface * InitSDL(int width, int height, std::string Title);

int main( int argc, char* args[] )
{
	//Initialize
	bool success;

	SDL_Surface *screen = InitSDL(640, 480, "Test Window");
	SDL_Event event;

	bool Quit = false;

	for(unsigned int i = 0; i < 10000; i++)
  {
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glEnable(GL_DEPTH_TEST);

    glColor3f(1.0f, 0.0f, 0.0f);  // Color Red
    Draw_Line(screen ,
               0, 0, 200, 300, // (x1, y1, x2, y2)
               0); //color
    SDL_GL_SwapBuffers();
  }

	SDL_Quit();

	return 0;
}

SDL_Surface * InitSDL(int width, int height, std::string Title)
{
  //Initialize all SDL subsystems
  int initok = SDL_Init( SDL_INIT_EVERYTHING );
  assert(initok != -1 );

  //Set up the screen
  const int SCREEN_BPP = 32; //or 16
  //SDL_Surface *screen = SDL_SetVideoMode( width, height, SCREEN_BPP, SDL_SWSURFACE );

  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1); // *new*
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);

  SDL_Surface *screen = SDL_SetVideoMode(width, height, SCREEN_BPP, SDL_OPENGL);

  //glClearColor(0, 0, 0, 0);//black
  glClearColor(1, 1, 1, 0); //white


  //If there was an error in setting up the screen
  assert(screen != NULL );

  //Set the window caption
  SDL_WM_SetCaption( Title.c_str(), NULL );

  return screen;
}