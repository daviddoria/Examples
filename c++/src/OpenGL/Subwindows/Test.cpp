#include <iostream>

#include "zpr.h"
#include <glut.h>
#include <glu.h>
#include <gl.h>

// g++ Test.cpp zpr.c -o Test -lglut -lGLU -lGL -lXmu -lX11 -I/usr/include/GL/ -I./ -L/usr/local/lib
using namespace std;

int MainID, Sub2dID, Sub3dID;

void OpenGLinit(int argc, char *argv[]);
int Create2dSubWindow(int MainID, const double x, const double y, const double w, const double h);
int Create3dSubWindow(int MainID, const double x, const double y, const double w, const double h);
void Display(void);
void DisplaySub3d(void);
void DisplaySub2d(void);
void DrawCube();

int main(int argc, char *argv[])
{
	
	OpenGLinit(argc, argv);
	MainID = glutCreateWindow("LidarSimulation");
	//glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);

	zprInit();
	
	glutDisplayFunc(Display);
	//glutKeyboardFunc(ProcessKeyboard);
	//glutSpecialFunc(ProcessSpecialKeys);
	zprSelectionFunc(Display); // Selection mode draw function
	//zprPickFunc(pick);
	
	//setup 2d subwindow
	//Sub2dID = Create2dSubWindow(MainID, 0, 0, 300, 200);
	//glutDisplayFunc(DisplaySub2d);
	//glutSetWindow(MainID);
	
	//setup 3d subwindow
	//Sub3dID = Create3dSubWindow(MainID, 0, 500, 300, 200);
	//glutDisplayFunc(DisplaySub3d);	
	//glutSetWindow(MainID);//return focus to main window
	
	glutMainLoop();

	return 0;
}

void Display(void)
{
	glutSetWindow(MainID);
	glClear(GL_COLOR_BUFFER_BIT);
		
	glLoadIdentity();
	
	gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0);
	glScalef(1, 2, 1);
	glutWireCube(1);
	
	//DrawCube();
	
	//glFlush();

/*
	DrawCube();
	
	glutSwapBuffers();
	
	//update the 2d subwindow
	glutPostWindowRedisplay(Sub2dID);

	//update the 3d subwindow
	glutPostWindowRedisplay(Sub3dID);
*/
	
}

void DisplaySub2d(void)
{
	glutSetWindow(Sub2dID);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	DrawCube();
	
	glutSwapBuffers();
}

void DisplaySub3d(void)
{
	glutSetWindow(Sub3dID);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

glPushMatrix();
	glRotated(90, 0, 1, 0);
	DrawCube();
glPopMatrix();		
	glutSwapBuffers();
}

void OpenGLinit(int argc, char *argv[])
{
	int WindowHeight = 1000;
	int WindowWidth = 1000;

	//Initialise GLUT and create a window
	glutInit(&argc, argv);
	//glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
	glutInitWindowSize(WindowWidth,WindowHeight);
	glutInitWindowPosition(0,0);
	
}

int Create2dSubWindow(int MainID, const double x, const double y, const double w, const double h)
{
	int SubWindowID = glutCreateSubWindow(MainID, x, y, w, h);

	//glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);

	//glMatrixMode(GL_MODELVIEW);
	glMatrixMode(GL_PROJECTION);
	//gluOrtho2D(0, w, 0, h);
	gluOrtho2D(-3, 3, -2, 2);

	return SubWindowID;
}



int Create3dSubWindow(int MainID, const double x, const double y, const double w, const double h)
{
	int SubWindowID = glutCreateSubWindow(MainID, x, y, w, h);
                                           
	glOrtho(-3, 3, -3, 3, -3, 3);//(left, right, bottom, top, near, far)
	
	//glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);
	
	
	//glMatrixMode(GL_PROJECTION);    
	
	
	//gluLookAt(0, 10, 0, 		  0, 0, 0,    		  0, 0, 1);

	//glMatrixMode(GL_MODELVIEW);                                           
	
	
	return SubWindowID;
}



void DrawCube()
{
	glBegin(GL_QUADS);
		glColor3f(1.0f,0.0f,0.0f); // Red
		glVertex3f(-1.0f, 1.0f, 1.0f);
		glVertex3f( 1.0f, 1.0f, 1.0f);
		glVertex3f( 1.0f, 1.0f,-1.0f);
		glVertex3f(-1.0f, 1.0f,-1.0f);
	
		glColor3f(0.0f,0.0f,1.0f); // Blue
		glVertex3f(-1.0f,-1.0f, 1.0f);
		glVertex3f(-1.0f,-1.0f,-1.0f);
		glVertex3f( 1.0f,-1.0f,-1.0f);
		glVertex3f( 1.0f,-1.0f, 1.0f);
	glEnd();
     glBegin(GL_TRIANGLE_STRIP);
	glColor3f(1.0f,1.0f,1.0f); // White
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f,-1.0f, 1.0f);
	glVertex3f( 1.0f, 1.0f, 1.0f);
	glColor3f(0.0f,1.0f,0.0f); // Green
	glVertex3f( 1.0f,-1.0f, 1.0f);
	glColor3f(1.0f,1.0f,0.0f); // Yellow
	glVertex3f( 1.0f, 1.0f,-1.0f);
	glColor3f(0.0f,1.0f,1.0f); // Aqua?
	glVertex3f( 1.0f,-1.0f,-1.0f);
	glColor3f(0.6f,0.6f,0.6f); // Gray
	glVertex3f(-1.0f, 1.0f,-1.0f);
	glColor3f(0.1f,0.1f,0.1f); // Dark Gray
	glVertex3f(-1.0f,-1.0f,-1.0f);
	glColor3f(0.0f,0.0f,1.0f); // Blue
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glColor3f(1.0f,0.0f,1.0f); // Red
	glVertex3f(-1.0f,-1.0f, 1.0f);
     glEnd();
}

