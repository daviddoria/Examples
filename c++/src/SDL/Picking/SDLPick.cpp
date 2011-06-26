#if 1
#include <SDL/SDL.h>
#include <SDL/SDL_opengl.h>

#include <stdio.h>
#include <iostream>
#include <GL/glut.h>
#include "SDL_Helper.h"

using namespace std;

void polygon(int a, int b, int c , int d);
void DrawCubeName();
void PickCenter(const int x, const int y, const int delX, const int delY);
void display();
void pick(GLint name);

int Width = 640, Height = 480;

int main(int argc, char *argv[])
{
	SDL_Surface *screen;
	
	assert( SDL_Init(SDL_INIT_VIDEO) == 0 );
	
	SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1 ); // *new*
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
	
	screen = SDL_SetVideoMode(Width, Height, 16, SDL_OPENGL); // *changed*
	assert(screen);
	//SDL_GetError());
	
	for(int i = 0; i < 72; i++)
	{
		display();
		PickCenter(Width/2, Height/2, 3, 3);
		//PickCenter(Width/2, Height/2, 600, 400);
		SDL_Delay(100);
    	}

	SDL_Quit();
    
	return 0;
}

int counter = 0;

bool ReallyDraw = true;

void DrawCrosshairs()
{
	// this should be the FIRST thing done when drawing
	
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	glOrtho(0.0f,Width,0.0f,Height,-10,100);
		
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
		
#define num 10
	unsigned char pixels[num][num];
	for(int i = 0; i < num; i++)
	{
		for(int j = 0; j < num; j++)
		{
			pixels[i][j] = 200;
		}
	}
	glRasterPos2i(Width/2 + 10, Height/2);
	glDrawPixels(num,num, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
	glRasterPos2i(Width/2 - 10, Height/2);
	glDrawPixels(num,num, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
	glRasterPos2i(Width/2, Height/2 + 10);
	glDrawPixels(num,num, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
	glRasterPos2i(Width/2, Height/2 - 10);
	glDrawPixels(num,num, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
		
}

void display()
{
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glEnable(GL_DEPTH_TEST);
	
	if(ReallyDraw)
	{
		DrawCrosshairs();
		
		glMatrixMode( GL_PROJECTION );
		glLoadIdentity();
		gluPerspective(20, 1, 1, 100); //field of view angle, aspect ratio, znear, zfar
    	
		glMatrixMode( GL_MODELVIEW );
		glLoadIdentity();
		gluLookAt(2, 2, 10, 0, 0, 0, 0, 1, 0);
	
		glRotatef(5*counter,0,1,0);
		//DrawCubeName();
		
		
		counter++;
	}
	//else
	
	DrawCubeName();

	SDL_GL_SwapBuffers();
}

void PickCenter(const int x, const int y, const int delX, const int delY)
{
	cout << "Center: x: " << x << " y: " << y << endl
			<< "Width: " << delX << " Height: " << delY << endl;
	
	unsigned int buffer[1024];
	const int bufferSize = sizeof(buffer)/sizeof(GLuint);
	
	GLint viewport[4];
	GLdouble projection[16];
	
	GLint hits;
	GLint i,j,k;
	
	GLint min  = -1;
	GLuint minZ = -1;
	
	glSelectBuffer(bufferSize, buffer);              /* Selection buffer for hit records */
	glRenderMode(GL_SELECT);                        /* OpenGL selection mode            */
	glInitNames();                                  /* Clear OpenGL name stack          */
	
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();                                 /* Push current projection matrix   */
	glGetIntegerv(GL_VIEWPORT,viewport);            /* Get the current viewport size    */
	glGetDoublev(GL_PROJECTION_MATRIX,projection);  /* Get the projection matrix        */
	glLoadIdentity();                               /* Reset the projection matrix      */
	gluPickMatrix(x,y,delX,delY,viewport);          /* Set the picking matrix           */
	glMultMatrixd(projection);                      /* Apply projection matrix          */
	
	glMatrixMode(GL_MODELVIEW);
	
	ReallyDraw = false;
	display();/* Draw the scene in selection mode */
	ReallyDraw = true;
	
	hits = glRenderMode(GL_RENDER);                 /* Return to normal rendering mode  */
	
	//debug output
	if (hits!=0)
	{
		printf("hits = %d\n",hits);
	
		for (i=0,j=0; i<hits; i++)
		{
			printf("\tsize = %u, min = %u, max = %u : ",buffer[j],buffer[j+1],buffer[j+2]);
			for (k=0; k < (GLint) buffer[j]; k++)
				printf("%u ",buffer[j+3+k]);
			printf("\n");
	
			j += 3 + buffer[j];
		}
	}
	
	/* Determine the nearest hit */
	
	if (hits)
	{
		for (i=0,j=0; i<hits; i++)
		{
			if (buffer[j+1]<minZ)
			{
			/* If name stack is empty, return -1                */
			/* If name stack is not empty, return top-most name */
		
				if (buffer[j]==0)
					min = -1;
				else
					min  = buffer[j+2+buffer[j]];
			
				minZ = buffer[j+1];
			}
			
			j += buffer[j] + 3;
		}
	}

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();                         /* Restore projection matrix           */
	glMatrixMode(GL_MODELVIEW);
	
	cout << "Name: " << min << endl << endl;

}
#endif

#if 0
#include <SDL/SDL.h>
#include <SDL/SDL_opengl.h>

#include <stdio.h>
#include <iostream>

using namespace std;

void polygon(int a, int b, int c , int d);
void DrawCubeName();
void PickCenter(GLdouble x, GLdouble y,GLdouble delX, GLdouble delY);
void display();
void pick(GLint name);

int main(int argc, char *argv[])
{
	SDL_Surface *screen;

    // Slightly different SDL initialization
	if ( SDL_Init(SDL_INIT_VIDEO) != 0 ) {
		printf("Unable to initialize SDL: %s\n", SDL_GetError());
		return 1;
	}

	SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1 ); // *new*
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);

    //screen = SDL_SetVideoMode( 640, 480, 16, SDL_OPENGL | SDL_FULLSCREEN ); // *changed*
	screen = SDL_SetVideoMode( 640, 480, 16, SDL_OPENGL); // *changed*
	if ( !screen ) {
		printf("Unable to set video mode: %s\n", SDL_GetError());
		return 1;
	}
	
    // Set the OpenGL state after creating the context with SDL_SetVideoMode

	glClearColor( 0, 0, 0, 0 );
	

	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective(70, 1, 1, 100); //field of view angle, aspect ratio, znear, zfar
    
    
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	gluLookAt(2, 2, 10, 0, 0, 0, 0, 1, 0);
   
	for(int i = 0; i < 72; i++)
	{
		display();
		PickCenter(320, 240, 3, 3);
		SDL_Delay(100);
    		
	}

	SDL_Quit();
    
	return 0;
}

int counter = 0;
bool ReallyDraw = true;

void display()
{
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glEnable(GL_DEPTH_TEST);
	
#if 0
	if(ReallyDraw)
	{
		glMatrixMode( GL_MODELVIEW );
		glLoadIdentity();
		gluLookAt(2, 2, 10, 0, 0, 0, 0, 1, 0);
	
		//glRotatef(5*counter,0,1,0);
		for(int i = 0; i < counter; i++)
			glRotatef(5,0,1,0);
		
		counter++;
	}
#else
	if(ReallyDraw)
	{
		glRotatef(5,0,1,0);
	}
#endif
	


	
	DrawCubeName();
		
	
	SDL_GL_SwapBuffers();
}



void PickCenter(GLdouble x, GLdouble y,GLdouble delX, GLdouble delY)
{
	GLuint buffer[1024];
	const int bufferSize = sizeof(buffer)/sizeof(GLuint);

	GLint    viewport[4];
	GLdouble projection[16];

	GLint hits;
	GLint i,j,k;

	GLint  min  = -1;
	GLuint minZ = -1;

	glSelectBuffer(bufferSize,buffer);              /* Selection buffer for hit records */
	glRenderMode(GL_SELECT);                        /* OpenGL selection mode            */
	glInitNames();                                  /* Clear OpenGL name stack          */

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();                                 /* Push current projection matrix   */
	glGetIntegerv(GL_VIEWPORT,viewport);            /* Get the current viewport size    */
	glGetDoublev(GL_PROJECTION_MATRIX,projection);  /* Get the projection matrix        */
	glLoadIdentity();                               /* Reset the projection matrix      */
	gluPickMatrix(x,y,delX,delY,viewport);          /* Set the picking matrix           */
	glMultMatrixd(projection);                      /* Apply projection matrix          */

	glMatrixMode(GL_MODELVIEW);


	//    selection();      
	ReallyDraw = false;                            
	display();
	ReallyDraw = true;
	hits = glRenderMode(GL_RENDER);                 /* Return to normal rendering mode  */

	/* Determine the nearest hit */

	if (hits)
	{
		for (i=0,j=0; i<hits; i++)
		{
			if (buffer[j+1]<minZ)
			{
				/* If name stack is empty, return -1                */
				/* If name stack is not empty, return top-most name */

				if (buffer[j]==0)
					min = -1;
				else
					min  = buffer[j+2+buffer[j]];

				minZ = buffer[j+1];
			}

			j += buffer[j] + 3;
		}
	}

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();                         /* Restore projection matrix           */
	glMatrixMode(GL_MODELVIEW);

	if (pick) //make sure a pick function is defined
		pick(min);                          /* Pass pick event back to application */
}

#if 0
void PickCenter(GLdouble x, GLdouble y,GLdouble delX, GLdouble delY)
{
	GLuint buffer[1024];
	const int bufferSize = sizeof(buffer)/sizeof(GLuint);

	GLint    viewport[4];
	GLdouble projection[16];

	GLint hits;
	GLint i,j,k;

	GLint  min  = -1;
	GLuint minZ = -1;

	glSelectBuffer(bufferSize,buffer);              /* Selection buffer for hit records */
	glRenderMode(GL_SELECT);                        /* OpenGL selection mode            */
	glInitNames();                                  /* Clear OpenGL name stack          */

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();                                 /* Push current projection matrix   */
	glGetIntegerv(GL_VIEWPORT,viewport);            /* Get the current viewport size    */
	glGetDoublev(GL_PROJECTION_MATRIX,projection);  /* Get the projection matrix        */
	glLoadIdentity();                               /* Reset the projection matrix      */
	gluPickMatrix(x,y,delX,delY,viewport);          /* Set the picking matrix           */
	glMultMatrixd(projection);                      /* Apply projection matrix          */

	glMatrixMode(GL_MODELVIEW);


	//    selection();      
	ReallyDraw = false;                            
	display();
	ReallyDraw = true;
	hits = glRenderMode(GL_RENDER);                 /* Return to normal rendering mode  */

	/* Determine the nearest hit */

	if (hits)
	{
		for (i=0,j=0; i<hits; i++)
		{
			if (buffer[j+1]<minZ)
			{
				/* If name stack is empty, return -1                */
				/* If name stack is not empty, return top-most name */

				if (buffer[j]==0)
					min = -1;
				else
					min  = buffer[j+2+buffer[j]];

				minZ = buffer[j+1];
			}

			j += buffer[j] + 3;
		}
	}

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();                         /* Restore projection matrix           */
	glMatrixMode(GL_MODELVIEW);

	if (pick) //make sure a pick function is defined
		pick(min);                          /* Pass pick event back to application */
}

#endif


void pick(GLint name)
{
	cout << "Name: " << name << endl;
}

#endif