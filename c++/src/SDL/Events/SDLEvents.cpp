/*This source code copyrighted by Lazy Foo' Productions (2004-2008) and may not
be redestributed without written permission.*/

//The headers
#include <SDL/SDL.h>
#include <SDL/SDL_image.h>
#include <string>
#include <iostream>
#include "SDL_Helper.h"
#include <GL/glut.h>
#include <assert.h>

using namespace std;

//The event structure that will be used


void apply_surface( int x, int y, SDL_Surface* source, SDL_Surface* destination );

bool HandleEvents(SDL_Event &event);

int main( int argc, char* args[] )
{

	//Initialize
	bool success;
	SDL_Surface *screen = InitSDL(640,480,"Test Window");

	/*
	//Load the files
	SDL_Surface *image = load_image( "x.png" );
	
	//Apply the surface to the screen
	apply_surface( 0, 0, image, screen );
	*/
	SDL_Event event;

	//Update the screen
	if( SDL_Flip( screen ) == -1 )
	{
		return 1;     //error
	}

	bool QuitSDL = false;
	
    /*
    while( QuitSDL == false )
    {
		while( SDL_PollEvent( &event ) )
	*/

	while(SDL_WaitEvent(&event))
	{
		QuitSDL = HandleEvents(event);
		if(QuitSDL)
		{
			break;
		}
	}
	
	SDL_FreeSurface(screen);
    SDL_Quit();

    return 0;
}

/*
void apply_surface( int x, int y, SDL_Surface* source, SDL_Surface* destination )
{
    //Temporary rectangle to hold the offsets
	SDL_Rect offset;
    
    //Get the offsets
	offset.x = x;
	offset.y = y;
    
    //Blit the surface
	SDL_BlitSurface( source, NULL, destination, &offset );
}
*/

bool HandleEvents(SDL_Event &event)
{
	//If the user has clicked the X (to close the window)
	if( event.type == SDL_QUIT )
	{
		//Quit the program
		cout << "Clicked the X" << endl;
		return true;
	}

	if(event.type == SDL_MOUSEBUTTONDOWN && SDL_BUTTON(SDL_GetMouseState(NULL,NULL)) == SDL_BUTTON_LEFT)
	{
		cout << "mouse left down" << endl;
	}
	
	if(event.type == SDL_MOUSEBUTTONUP) // returns bullet to player
	{
		cout << "Mouse up" << endl;
	}
	
	if (event.type == SDL_KEYDOWN) //you must enclose key checks in this KEYDOWN check
	{
		if (event.key.keysym.sym == SDLK_ESCAPE)
		{
			cout << "pressed esc" << endl;
			return true;
		}

		if (event.key.keysym.sym == SDLK_a)
		{
			cout << "pressed a" << endl;
		}

	}

	if (event.type == SDL_KEYUP)
	{

		if (event.key.keysym.sym == SDLK_a)
		{
			cout << "released a" << endl;
		}


	}

	if(event.type == SDL_MOUSEMOTION)
	{
		cout << "x: " << event.motion.x << "y: " << event.motion.y << endl;
	}

	return false;
}