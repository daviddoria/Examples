#ifndef MOUSECLASS_H
#define MOUSECLASS_H

#include <iostream>

using namespace std;
	
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