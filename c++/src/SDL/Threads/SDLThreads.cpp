/*This source code copyrighted by Lazy Foo' Productions (2004-2008) and may not
be redestributed without written permission.*/

//The headers
#include <SDL/SDL.h>
#include <SDL/SDL_thread.h>

#include <string>
#include <iostream>
#include <SDL/SDL_thread.h>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <vector>

#include <boost/progress.hpp>

#include <SDL/SDL_opengl.h>
#include <GL/glx.h>
#include <GL/glxext.h>

using namespace std;

void LongFunction();
void SDL();
int event_thread(void *arg);

int main(int argc, char **argv)
{
	atexit(SDL_Quit);
	SDL();

	SDL_Thread *thread;
	thread = SDL_CreateThread(event_thread, NULL);
	LongFunction();

	return 0;
}

void LongFunction()
{
	unsigned int BigNum = 1e9;
	boost::progress_display show_progress(BigNum);
	
	double temp;
	for(unsigned int i = 0; i < BigNum; i++)
	{
		temp = sin(i) / i;
		++show_progress;
	}
}

void SDL()
{
	SDL_Surface *screen;
	if ( SDL_Init(SDL_INIT_VIDEO) < 0 ) 
	{
		fprintf(stderr, "Unable to init SDL: %s\n", SDL_GetError());
		exit(1);
	}
	atexit(SDL_Quit);

	SDL_GL_SetAttribute(SDL_GL_RED_SIZE,     8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE,   8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE,    8);
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

	screen = NULL;
	int width = 1024;
	int height = 1024;
	if (NULL == (screen = SDL_SetVideoMode(width, height, 0, SDL_OPENGL)))
	{
		fprintf(stderr, "Can't set OpenGL mode: %s\n", SDL_GetError());
		SDL_Quit();
		exit(1);
	}

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, 3, 3, 0, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity(); 
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glLineWidth(2.0);
	glPointSize(2.0);
}

int event_thread(void *arg)
{
	bool done = false;
	SDL_Event event;
	while (!done)
	{
		while (!done && SDL_WaitEvent(&event))
		{
			switch (event.type)
			{
				case SDL_KEYDOWN:
					switch(event.key.keysym.sym)
					{
						case SDLK_ESCAPE:
							done = true;
							break;
						default:
							break;
					}
					break;

				case SDL_QUIT:
					done = true;
					break;
					
				case SDL_MOUSEMOTION:
					cout << "x: " << event.motion.x << "y: " << event.motion.y << endl;
					break;
			}
		}
	}
	exit(0);
}

