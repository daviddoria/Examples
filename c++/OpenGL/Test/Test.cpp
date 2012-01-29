#include <iostream>
#include <cstdlib> //for exit()

#include <GL/glut.h>

#include <vil/vil_rgb.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>
#include <vil/vil_image_view.h>
#include <vil/vil_convert.h>

void display(void);
void ProcessKeyboard(unsigned char key, int x, int y);
void DrawCube();
void DrawSizedCube(const float s);
void polygon(int a, int b, int c , int d);
void DrawPoint(double x, double y, double z);
void LookAt();
void WriteRGBImage();

const unsigned int Width = 100;
const unsigned int Height = 100;

int main(int argc, char *argv[])
{
	std::cout << "OpenGL" << std::endl;

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(Width, Height);
	glutInitWindowPosition(0, 0);
	
	//setup the main window
	glutCreateWindow("OpenGL Example");
	glutHideWindow();
	
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

bool Done = false;

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

	if(!Done)
	{
		WriteRGBImage();
		Done = true;
	}
	
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

void WriteRGBImage()
{	
	vil_image_view<vil_rgb<vxl_byte> > RGBImage(Width, Height, 1, 1);

	static GLubyte bufImage[Width][Height][3];
	//GLubyte bufImage[100][100][3];
	
	//GLubyte r,g,b;
	unsigned char r,g,b;
	
	glReadBuffer(GL_FRONT);
	glReadPixels(0, 0, Width, Height, GL_RGB, GL_UNSIGNED_BYTE, bufImage);
	
	for(unsigned int im_x = 0; im_x < Width; im_x++)
	{
		for(unsigned int im_y = 0; im_y < Height; im_y++)
		{
			r = bufImage[im_x][im_y][0];
			g = bufImage[im_x][im_y][1];
			b = bufImage[im_x][im_y][2];
			//RGBImage(im_x, im_y) = vil_rgb<vxl_byte>(r,g,b);
			RGBImage(im_x, im_y) = vil_rgb<vxl_byte>(r,g,b);
		}
	}

	vil_save(RGBImage, "RGBImage.jpg");

	std::cout << "Done writing RGB image." << std::endl;
}