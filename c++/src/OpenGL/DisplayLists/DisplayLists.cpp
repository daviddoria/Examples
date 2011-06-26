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

  //setup the main window
  glutCreateWindow("OpenGL Example");

  glutDisplayFunc(display);

  DrawCube(); //create display list

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(70, 1, 1, 100);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  gluLookAt(2, 2, 10, 2, 0, 0, 0, 1, 0);


  glutMainLoop();
  return 0;
}

GLuint list;

void display(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glEnable(GL_DEPTH_TEST);

  glCallList(list);

  glutSwapBuffers();
}


void DrawCube()
{
  list = glGenLists(1);
  glNewList(list,GL_COMPILE);

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

  glEndList();
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
