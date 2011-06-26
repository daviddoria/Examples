#include <iostream>
#include <cstdlib> //for exit()
#include <string>

//#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <vil/vil_rgb.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>
#include <vil/vil_image_view.h>
#include <vil/vil_convert.h>

#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_vector_3d.h>

using namespace std;

void display(void);

void DrawCube();

void polygon(const unsigned int a, const unsigned int b, const unsigned int c , const unsigned int d);
void DrawPoint(const double x, const double y, const double z);
void LookAt();
void ReadDepthBuffer();
void WriteDepthFileVIL(float* depths);
void WriteRGBImage();

//unsigned int Width = 100;
//unsigned int Height = 100;
#define Width 100
#define Height 100

int main(int argc, char *argv[])
{

	GLenum error;
	error = glGetError();
	cout << gluErrorString(error) << endl;
	
	display();

	error = glGetError();
	cout << gluErrorString(error) << endl;
	
	return 0;
}

void DrawPoint(const double x, const double y, const double z)
{
	glPointSize(10.0);
	glBegin(GL_POINTS);
		glColor3f(1.0, 0.0, 0.0);//red
		glVertex3f(x,y,z);
	glEnd();
}


void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	LookAt();
	DrawCube();
	ReadDepthBuffer();
	WriteRGBImage();
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


void polygon(const unsigned int a, const unsigned int b, const unsigned int c , const unsigned int d)
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


void ReadDepthBuffer()
{
	cout << "Reading..." << endl;
	
	float depths[Width*Height];
	glReadPixels(0, 0, Width, Height, GL_DEPTH_COMPONENT, GL_FLOAT, depths);

	WriteDepthFileVIL(depths);
	
	
}


void WriteDepthFileVIL(float* depths)
{	
	double ModelView[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, ModelView);

	double Projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX, Projection);

	GLint Viewport[4];
	glGetIntegerv(GL_VIEWPORT, Viewport);

	double x;
	double y;
	double z;

	vil_image_view<unsigned char> DepthBuffer(Width, Height, 1, 1);
	vil_image_view<unsigned char> DepthImage(Width, Height, 1, 1);
	
	for(unsigned int im_x = 0; im_x < Width; im_x++)
	{
		for(unsigned int im_y = 0; im_y < Height; im_y++)
		{
			double CurrentDepth = depths[Width*im_y + im_x];
			DepthBuffer(im_x, im_y) = 255 * CurrentDepth;

			gluUnProject(static_cast<double>(im_x), static_cast<double>(im_y), CurrentDepth, ModelView, Projection, Viewport, &x, &y, &z);
			vgl_point_3d<double> ScannerLocation(0.0, 0.0, 0.0);
			vgl_point_3d<double> Point(x,y,z);

			// !!! Need to normalize (0,1) for image to make sense
			DepthImage(im_x, im_y) = (Point - ScannerLocation).length();
		}
	}

	vil_save(DepthBuffer, "DepthBuffer.jpg");
	vil_save(DepthImage, "DepthImage.jpg");

	cout << "Done writing images." << endl;
}


void WriteRGBImage()
{	
	vil_image_view<vil_rgb<vxl_byte> > RGBImage(Width, Height, 1, 1);

	static GLubyte bufImage[Width][Height][3];
	//GLubyte bufImage[100][100][3];
	
	//GLubyte r,g,b;
	unsigned char r,g,b;
	
	//glReadBuffer(GL_FRONT);
	glReadBuffer(GL_BACK);
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

	cout << "Done writing RGB image." << endl;
}