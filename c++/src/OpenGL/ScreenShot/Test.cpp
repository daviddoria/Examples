#include <iostream>
#include <cstdlib> //for exit()

#include <GL/glut.h>

#include <vil/vil_rgb.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>
#include <vil/vil_image_view.h>
#include <vil/vil_convert.h>

#include "Screenshot.h"

using namespace std;

void display(void);
void ProcessKeyboard(unsigned char key, int x, int y);
void DrawCube();
void DrawSizedCube(const float s);
void polygon(int a, int b, int c , int d);
void LookAt();

const unsigned int Width = 200;
const unsigned int Height = 100;

int main(int argc, char *argv[])
{
	cout << "OpenGL" << endl;

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(Width, Height);
	glutInitWindowPosition(0, 0);
	
	//setup the main window
	glutCreateWindow("OpenGL Example");
	
	glutDisplayFunc(display);
	glutKeyboardFunc(ProcessKeyboard);

	glutMainLoop();
	return 0;
}


void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	LookAt();
	DrawCube();

	glutSwapBuffers();
	
}


void ProcessKeyboard(unsigned char key, int x, int y)
{
	if (key == 27) //escape
	{
		exit(0);
	}
	else if(key == 'q')
	{
		exit(0);
	}
	else if(key == 's')
	{
		cout << "Screenshot." << endl;
		Screenshot("Test.jpg");
		//ScreenshotFixed("Fixed.jpg");
	}
	
	display();
}

void DrawSizedCube(const float s)
{
	glPushMatrix();
		glScalef(s,s,s);
		DrawCube();
	glPopMatrix();

}

void DrawCube()
{
	//back
	glColor3f(1.0f, 0.0f, 0.0f);	// Color Red
	polygon(0,3,2,1);
	
	//top
	glColor3f(0.0f,1.0f,0.0f);	// Color Green
	polygon(2,3,7,6);
	
	//left
	glColor3f(0.0f, 0.0f, 1.0f);	// Color Blue
	polygon(0,4,7,3);
	
	//right
	glColor3f(1.0f,1.0f,0.0f);	// Color Yellow
	polygon(1,2,6,5);
	
	//front
	glColor3f(0.0f,1.0f,1.0f);	// Color Cyan 
	polygon(4,5,6,7);
	
	//bottom
	glColor3f(1.0f,0.0f,1.0f);	// Color Magenta
	polygon(0,1,5,4);
	
}


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


void LookAt()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(70, 1, 1, 100);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	gluLookAt(3, 5, 5, 0, 0, 0, 0, 1, 0);
	
}

