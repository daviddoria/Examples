#include <SDL/SDL.h>
#include <SDL/SDL_image.h>

#include <string>
#include <iostream>
#include <fstream>
#include "SDL_Helper.h"
#include <GL/glut.h>
#include <assert.h>

#include <vnl/vnl_double_3.h>

using namespace std;

void display();
void LookAt();
bool HandleEvents(SDL_Event &event);
void ReadDepthBuffer();
void WriteDepthFile(const string &Filename, float* depths);

unsigned int Width = 640, Height = 480;
double EyeX = 5.0;
double EyeY = 0.0;
double EyeZ = 5.0;

int main( int argc, char* args[] )
{
	cout << "main" << endl;

	//Initialize
	bool success;

	SDL_Surface *screen = InitSDL(Width, Height, "Test Window");
	SDL_Event event;

	bool Quit = false;

	for(unsigned int i = 0; i < 10000; i++)
		display();

	SDL_Quit();

	return 0;
}


void display()
{
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glEnable(GL_DEPTH_TEST);

	LookAt();
	DrawCube();

	SDL_GL_SwapBuffers();
}


void LookAt()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(70, 1, 1, 10); //view angle in y direction, aspect (width to height), zNear, zFar
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	gluLookAt(2.0, 2.0, 10.0, 2.0, 0.0, 0.0, 0.0, 1.0, 0.0); //eyex, eyey, eyez, centerofviewx, centerofviewy, centerofviewz, upx, upy, upz

}
