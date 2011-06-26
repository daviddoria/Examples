#include <iostream>
#include <glut.h>

using namespace std;

void display();
void LookAt();
void DrawCube();
void DrawSphere();
void polygon(int a, int b, int c , int d);

int WindowHeight = 1000;
int WindowWidth = 1000;
	

int main(int argc, char *argv[])
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(WindowWidth, WindowHeight);
	glutInitWindowPosition(0, 0);
	
	glutCreateWindow("Picking Example");
	
	glutDisplayFunc(display);

	LookAt();
	glutMainLoop();
	return 0;
}

void DrawSphere()
{
	//glTranslatef(Position_.getX(), Position_.getY(), Position_.getZ());
	//glScalef(Diameter_, Diameter_, Diameter_);
					
	GLUquadricObj *quadratic;
	quadratic = gluNewQuadric();
	gluQuadricNormals(quadratic, GLU_SMOOTH);
	
	gluSphere(quadratic, 1.0f, 32, 32);
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
	
	//DrawCube();
	DrawSphere();
	
	glutSwapBuffers();
}

void MakeColorMap()
{
	/*
	map <string, int> MyMap;
	MyMap["testone"] = 1;
	MyMap["testtwo"] = 2;
	MyMap["testseventyeight"] = 78;
	
	cout << MyMap["testtwo"] << endl;
	cout << MyMap["testseventyeight"] << endl;
	
	cout << MyMap["not in map"] << endl;
	*/
}

GLfloat vertices[][3] = {{-1.0,-1.0,-1.0},{1.0,-1.0,-1.0},
		{1.0,1.0,-1.0}, {-1.0,1.0,-1.0}, {-1.0,-1.0,1.0}, 
  {1.0,-1.0,1.0}, {1.0,1.0,1.0}, {-1.0,1.0,1.0}};

  
void DrawCubeFloat()
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


void DrawCubeChar()
{

	glColor3ub(255, 0, 0);	// Color Red
	polygon(0,3,2,1);

	glColor3ub(0,255,0);	// Color Green
	polygon(2,3,7,6);

	glColor3ub(0, 0, 255);	// Color Blue
	polygon(0,4,7,3);

	glColor3ub(255,255,0);	// Color Yellow
	polygon(1,2,6,5);

	glColor3ub(0,255,255);	// Color Cyan 
	polygon(4,5,6,7);

	glColor3ub(255,0,255);	// Color Magenta
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

