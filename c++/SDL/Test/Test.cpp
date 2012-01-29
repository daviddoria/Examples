#include<SDL/SDL.h>

void apply_surface(int x,int y,SDL_Surface* source,SDL_Surface* dest)
{
	SDL_Rect offset;
	offset.x = x;
	offset.y = y;
	SDL_BlitSurface(source,NULL,dest,&offset);
}


int main( int argc, char** argv )
{
	SDL_Init(SDL_INIT_EVERYTHING);
	SDL_Surface* screen = SDL_SetVideoMode(500,500,32,0);
	
	SDL_LockSurface(screen);
	SDL_UnlockSurface(screen);

	SDL_Flip(screen);

	SDL_Delay(1000);

	SDL_Quit();
	return 0;
}