#include <iostream>
#include <string>
#include <sstream>
//#include <cstdlib>
#include <stdio.h>

#include <glut.h>

using namespace std;

//choose which buffer to draw to
//glDrawBuffer(GL_BACK);
//glDrawBuffer(GL_FRONT);


int counter = 0;


GLfloat vertices[][3] = {{-1.0,-1.0,-1.0},{1.0,-1.0,-1.0},
	{1.0,1.0,-1.0}, {-1.0,1.0,-1.0}, {-1.0,-1.0,1.0}, 
 {1.0,-1.0,1.0}, {1.0,1.0,1.0}, {-1.0,1.0,1.0}};

void display();
void LookAt();
void DrawCube();
void printtext(int x, int y, string String);
void printtext3d(const int x, const int y, const int z);
void MouseButtons(int button, int state, int x, int y);

void polygon(int a, int b, int c , int d);

int WindowHeight = 1000;
int WindowWidth = 1000;
	
GLvoid Timer(int value)
{
	glRotatef(3,0,1,0);
	glutPostRedisplay();
	glutTimerFunc(40,Timer,value);
}


int main(int argc, char *argv[])
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(WindowWidth, WindowHeight);
	//glutInitWindowPosition(0, 0);
	
	glutCreateWindow("Picking Example");
	
	glutDisplayFunc(display);
	glutMouseFunc(MouseButtons);
	LookAt();
	glutMainLoop();
	return 0;
}

void LookAt()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(70, 1, 1, 100);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	gluLookAt(2, 2, 10, 2, 0, 0, 0, 1, 0);
	
}

void display()
{
	//MUST do these
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	
	DrawCube();
	
	glDisable(GL_DEPTH_TEST);
	
	if(counter < 8)
		printtext3d(vertices[counter][0], vertices[counter][1], vertices[counter][2]);
		
	glutSwapBuffers();
}



void DrawCube()
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

void polygon(int a, int b, int c , int d)
{
	glBegin(GL_QUADS);
		glVertex3fv(vertices[a]);
		glVertex3fv(vertices[b]);
		glVertex3fv(vertices[c]);
		glVertex3fv(vertices[d]);
	glEnd();
}



void printtext3d(const int x, const int y, const int z)
{
	GLenum error;
	int rasterpos[4];
	
	
	GLboolean val[2];
	glGetBooleanv(GL_CURRENT_RASTER_POSITION_VALID, val);
	cout << "valid: " << (bool)val[0] << endl;
	

	glRasterPos3i(x,y,z);
	glGetIntegerv(GL_CURRENT_RASTER_POSITION, rasterpos);
	cout << "raster pos x: " << rasterpos[0] << " y: " << rasterpos[1] << " z: " << z << endl;
	
	
	glColor3f(1.0f, 1.0f, 1.0f);
	
	string TextString = "hello";
	for (int i=0; i < TextString.size(); i++)
	{
		glutBitmapCharacter(GLUT_BITMAP_9_BY_15, TextString[i]);
	}
	
	cout << "Drew text." << endl;
	
		
}

void MouseButtons(int button, int state, int x, int y)
{	
	if( (state == GLUT_DOWN) && (button == GLUT_LEFT_BUTTON) )
	{
		double newy = glutGet(GLUT_WINDOW_HEIGHT) - 1 - y;
		
		stringstream TextStream;
		TextStream << "Something";
		

		counter++;
		
		cout << "clicked." << endl;
	}
	
	glutPostRedisplay();
}