#include <iostream>

#include <GL/glut.h>

using namespace std;

void display(void);
void polygon(int a, int b, int c , int d);
void DrawCube();
		
int main(int argc, char *argv[])
{
	int WindowHeight = 1000;
	int WindowWidth = 1000;
	
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(WindowWidth, WindowHeight);
	glutInitWindowPosition(0, 0);
	
	glutCreateWindow("OpenGL Example");
	
	glutDisplayFunc(display);

	glutMainLoop();
	return 0;
}



void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glEnable(GL_DEPTH_TEST);
	
	DrawCube();
	
	GLenum error = glGetError();
	
	cout << gluErrorString(error) << endl;
	
	glutSwapBuffers();
}


void DrawCube()
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
