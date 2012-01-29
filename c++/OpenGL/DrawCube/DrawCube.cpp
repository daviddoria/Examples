#include <iostream>
#include <cstdlib> //for exit()

#include <GL/glut.h>

void display(void);
void ProcessKeyboard(unsigned char key, int x, int y);
void DrawCube();
void DrawSizedCube(const float s);
void polygon(int a, int b, int c , int d);
void DrawPoint(double x, double y, double z);
void LookAt();

const unsigned int Width = 1000;
const unsigned int Height = 1000;

int main(int argc, char *argv[])
{
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(Width, Height);
  glutInitWindowPosition(0, 0);

  // Setup the main window
  glutCreateWindow("OpenGL Example");

  glutDisplayFunc(display);

  glutMainLoop();
  return 0;
}

void DrawPoint(double x, double y, double z)
{
  glPointSize(10.0);
  glBegin(GL_POINTS);
    glColor3f(1,0,0);//red
    glVertex3f(x,y,z);
  glEnd();
}

void display(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);

  LookAt();
  DrawCube();
  //DrawSizedCube(.5);
  //DrawPoint(1.0, 2.0, 1.0);
  //DrawPoint(0.0, 0.0, 0.0);

  glutSwapBuffers();

}

void ProcessKeyboard(unsigned char key, int x, int y)
{
  if (key == 27) //escape
  {
    exit(-1);
  }
  else if(key == 'q')
  {
    exit(-1);
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


void DrawAxes(void)
{

  glPushMatrix();
  /* No name for grey sphere */

  glColor3f(0.3,0.3,0.3);
  glutSolidSphere(0.7, 20, 20);

  glPushMatrix();
    glPushName(1);            /* Red cone is 1 */
      glColor3f(1,0,0);
      glRotatef(90,0,1,0);
      glutSolidCone(0.6, 4.0, 20, 20);
    glPopName();
  glPopMatrix();

  glPushMatrix ();
    glPushName(2);            /* Green cone is 2 */
      glColor3f(0,1,0);
      glRotatef(-90,1,0,0);
      glutSolidCone(0.6, 4.0, 20, 20);
    glPopName();
  glPopMatrix();

  glColor3f(0,0,1);         /* Blue cone is 3 */
  glPushName(3);
  glutSolidCone(0.6, 4.0, 20, 20);
  glPopName();

  glPopMatrix();
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

  //gluLookAt(2, 2, 10, 2, 0, 0, 0, 1, 0);
  gluLookAt(0, 5, 5, 0, 0, 0, 0, 1, 0);
}
