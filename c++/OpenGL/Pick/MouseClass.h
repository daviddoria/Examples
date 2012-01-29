#ifndef MOUSECLASS_H
#define MOUSECLASS_H

//glutMouseFunc(MouseButtons);
//glutMotionFunc(MouseMotion);
	
#include <iostream>

#include "Vector.h"


using namespace std;
	
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


void DrawCrosshairs(int x, int y, int Width, int Height)
{

#define num 10
	unsigned char pixels[num][num];
	for(int i = 0; i < num; i++)
	{
		for(int j = 0; j < num; j++)
		{
			pixels[i][j] = 200;
		}
	}
	
//	glWindowPos2i(x, y); // in GLEE
	glDrawPixels(num,num, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
	
//	glWindowPos2i(x + 100, y); // in GLEE
	glDrawPixels(num,num, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
	
}

class MouseClass
{
	public:
		int X;
		int Y;
		bool Left;
		bool Middle;
		bool Right;
		MouseClass() : X(0), Y(0), Left(false), Middle(false), Right(false){}
};

extern MouseClass Mouse;

void MouseButtons(int button, int state, int x, int y)
{
	int verbosity = 1;
	
	Mouse.X = x;
	Mouse.Y = y;

	if(state == GLUT_UP)
	{
		switch(button)
		{
			case GLUT_LEFT_BUTTON:   
				Mouse.Left = false;
				if(verbosity >= 1)
					cout << "Left Button Up" << endl;
				break;
			case GLUT_MIDDLE_BUTTON:
				Mouse.Middle = false;
				if(verbosity >= 1)
					cout << "Middle Button Up" << endl;
				break;
			case GLUT_RIGHT_BUTTON:
				Mouse.Right = false;
				if(verbosity >= 1)
					cout << "Right Button Up" << endl;
				break;
				
			
		}
	}
	else
	{
		switch(button)
		{
			case GLUT_LEFT_BUTTON:
				Mouse.Left = true;
				if(verbosity >= 1)
					cout << "Left Button Down" << endl;
				break;
			case GLUT_MIDDLE_BUTTON:
				Mouse.Middle = true;
				if(verbosity >= 1)
					cout << "Middle Button Down" << endl;
				break;
			case GLUT_RIGHT_BUTTON:
				Mouse.Right = true;
				if(verbosity >= 1)
					cout << "Right Button Down" << endl;
				break;
		}
	}
}

void MouseMotion(int x, int y)
{
	int verbosity = 0;
		
	const int dx = x - Mouse.X;
	const int dy = y - Mouse.Y;

	if ( (dx == 0) && (dy == 0) )
		return;

	if (Mouse.Middle) //zoom
	{
	}
	else if (Mouse.Left) //rotate
	{
	}
	else if (Mouse.Right) //translate
	{
	}

	Mouse.X = x;
	Mouse.Y = y;

	glutPostRedisplay();
}

#endif