#include "SDL_Helper.h"

SDL_Surface * InitSDL(int width, int height, string Title)
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


SDL_Surface *load_image(string filename) 
{
	//The image that's loaded
	SDL_Surface* loadedImage = NULL;
	
	//The optimized image that will be used
	SDL_Surface* optimizedImage = NULL;
	
	//Load the image
	loadedImage = IMG_Load( filename.c_str() );
	
	//If the image loaded
	if( loadedImage != NULL )
	{
	//Create an optimized image
		optimizedImage = SDL_DisplayFormat( loadedImage );
	
	//Free the old image
		SDL_FreeSurface( loadedImage );
	}
	
	assert(optimizedImage != NULL );
	
	//Return the optimized image
	return optimizedImage;
}

void DrawCube()
{
	glColor3f(1.0f, 0.0f, 0.0f);	// Color Red
	polygon(0,3,2,1);
		
	glColor3f(0.0f,1.0f,0.0f);	// Color Green
	polygon(2,3,7,6);
		
	glColor3f(0.0f, 0.0f, 1.0f);	// Color Blue
	polygon(0,4,7,3);
			
	glColor3f(1.0f,1.0f,0.0f);	// Color Yellow
	polygon(1,2,6,5);
		
	glColor3f(0.0f,1.0f,1.0f);	// Color Cyan
	polygon(4,5,6,7);
		
	glColor3f(1.0f,0.0f,1.0f);	// Color Magenta
	polygon(0,1,5,4);
	
}

void DrawCubeName()
{
	glPushName(1);
	
	glLoadName(1);
	glColor3f(1.0f, 0.0f, 0.0f);	// Color Red
	polygon(0,3,2,1);
		
	glLoadName(2);
	glColor3f(0.0f,1.0f,0.0f);	// Color Green
	polygon(2,3,7,6);
		
	glLoadName(3);
	glColor3f(0.0f, 0.0f, 1.0f);	// Color Blue
	polygon(0,4,7,3);
			
	glLoadName(4);
	glColor3f(1.0f,1.0f,0.0f);	// Color Yellow
	polygon(1,2,6,5);
		
	glLoadName(5);
	glColor3f(0.0f,1.0f,1.0f);	// Color Cyan 
	polygon(4,5,6,7);
		
	glLoadName(6);
	glColor3f(1.0f,0.0f,1.0f);	// Color Magenta
	polygon(0,1,5,4);
	
}
/*
void DrawCubeName()
{
	glPushName(1);
	glColor3f(1.0f, 0.0f, 0.0f);	// Color Red
	polygon(0,3,2,1);
	glPopName();
	
	glPushName(2);
	glColor3f(0.0f,1.0f,0.0f);	// Color Green
	polygon(2,3,7,6);
	glPopName();
	
	glPushName(3);
	glColor3f(0.0f, 0.0f, 1.0f);	// Color Blue
	polygon(0,4,7,3);
	glPopName();
		
	glPushName(4);
	glColor3f(1.0f,1.0f,0.0f);	// Color Yellow
	polygon(1,2,6,5);
	glPopName();
	
	glPushName(5);
	glColor3f(0.0f,1.0f,1.0f);	// Color Cyan 
	polygon(4,5,6,7);
	glPopName();
	
	glPushName(6);
	glColor3f(1.0f,0.0f,1.0f);	// Color Magenta
	polygon(0,1,5,4);
	glPopName();
}
*/

GLfloat vertices[][3] = {{-1.0,-1.0,-1.0},{1.0,-1.0,-1.0},
			{1.0,1.0,-1.0}, {-1.0,1.0,-1.0}, {-1.0,-1.0,1.0}, 
			{1.0,-1.0,1.0}, {1.0,1.0,1.0}, {-1.0,1.0,1.0}};

  
void polygon(int a, int b, int c , int d)
{
	glBegin(GL_QUADS);
		glVertex3fv(vertices[a]);
		glVertex3fv(vertices[b]);
		glVertex3fv(vertices[c]);
		glVertex3fv(vertices[d]);
	glEnd();
}

void DrawPoint(double x, double y, double z)
{
	glPointSize(10.0);
	glBegin(GL_POINTS);
		
	glColor3f(1,0,0);//red
	glVertex3f(x,y,z);
	glEnd();
}