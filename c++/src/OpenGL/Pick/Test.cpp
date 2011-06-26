#include <iostream>

#include <glut.h>

using namespace std;

void display();
void LookAt();
void DrawCube();

#define PICK_BUFFER_SIZE 256
unsigned int PickBuffer[PICK_BUFFER_SIZE];

int Pick(GLdouble x, GLdouble y,GLdouble delX, GLdouble delY);

void MouseButtonsPick(int button, int state, int x, int y);
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
	glutInitWindowPosition(0, 0);
	
	glutCreateWindow("Picking Example");
	
	glutDisplayFunc(display);
	glutMouseFunc(MouseButtonsPick);

	//glutTimerFunc(40,Timer,0);

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
	
	//LookAt(); //do NOT do this here
	
	//glMatrixMode(GL_MODELVIEW);
	
	DrawCube();
	
	glutSwapBuffers();
}



GLfloat vertices[][3] = {{-1.0,-1.0,-1.0},{1.0,-1.0,-1.0},
		{1.0,1.0,-1.0}, {-1.0,1.0,-1.0}, {-1.0,-1.0,1.0}, 
  {1.0,-1.0,1.0}, {1.0,1.0,1.0}, {-1.0,1.0,1.0}};

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


void MouseButtonsPick(int button, int state, int x, int y)
{	
	if( (state == GLUT_DOWN) && (button == GLUT_LEFT_BUTTON) )
	{
		int hits;
		double newy = glutGet(GLUT_WINDOW_HEIGHT) - 1 - y;
		cout << "x: " << x << " y: " << y << " newy: " << newy << endl;
		
		int name = Pick(x, newy, 3, 3);
		cout << "Name: " << name << endl;
	}
}

int Pick(GLdouble x, GLdouble y,GLdouble delX, GLdouble delY)
{
	GLuint buffer[1024];
	const int bufferSize = sizeof(buffer)/sizeof(GLuint);

	GLint    viewport[4];
	GLdouble projection[16];

	GLint hits;
	GLint i,j,k;

	GLint  min  = -1;
	GLuint minZ = -1;

	glSelectBuffer(bufferSize,buffer);              // Selection buffer for hit records
	glRenderMode(GL_SELECT);                        // OpenGL selection mode           
	glInitNames();                                  // Clear OpenGL name stack         

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();                                 // Push current projection matrix  
	glGetIntegerv(GL_VIEWPORT,viewport);            // Get the current viewport size   
	glGetDoublev(GL_PROJECTION_MATRIX,projection);  // Get the projection matrix       
	glLoadIdentity();                               // Reset the projection matrix     
	gluPickMatrix(x,y,delX,delY,viewport);          // Set the picking matrix          
	glMultMatrixd(projection);                      // Apply projection matrix         

	glMatrixMode(GL_MODELVIEW);

	display();

	hits = glRenderMode(GL_RENDER);

	if (hits)
	{
		for (i=0,j=0; i<hits; i++)
		{
			if (buffer[j+1]<minZ)
			{
				// If name stack is empty, return -1                
				// If name stack is not empty, return top-most name 

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
	glPopMatrix();       
	glMatrixMode(GL_MODELVIEW);

	return min;
}
