#include <iostream>
#include <fstream>
#include <cstdlib> //for exit()
#include <assert.h>


//MUST BE IN THIS ORDER!!
#define GL_GLEXT_PROTOTYPES
#include <GL/glut.h>
#include <GL/glext.h>

#include <SDLHelpers/SDLHelpers.h>

using namespace std;

void LookAt();
const unsigned int width = 100, height = 100;

/*
void CheckErrors();
void polygon(int a, int b, int c , int d);
void DrawCube();
*/

int main(int argc, char *argv[])
{
	cout << "OpenGL" << endl;
	
	
	//setup an opengl context
	//glXCreateContext(display, visual_info, NULL, true);
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	glutInitWindowPosition(0, 0);
	
	//setup the main window
	glutCreateWindow("OpenGL Example");
	glutHideWindow();

	//Create handle to FBO
	GLuint fbo;
	glGenFramebuffersEXT(1, &fbo);
	
	//bind FBO (what does this mean?)
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
	
	//create depth buffer for the FBO
	GLuint depthbuffer;
	glGenRenderbuffersEXT(1, &depthbuffer);
	
	//bind the renderbuffer
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depthbuffer);
	
	//specify size of the depth buffer
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, width, height);
	
	//attach the depth buffer to the frame buffer
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, depthbuffer);
	
	glDrawBuffer(GL_NONE);
	//check that everything went ok
	GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
	
	if(status != GL_FRAMEBUFFER_COMPLETE_EXT)
	{
		//it's not working!
		cout << "Not working!" << endl;
		cerr << "ERROR: Framebuffer creation failed with status "<< hex <<status <<dec <<endl;
		//cout << gluErrorString(status) << endl;
	}
	else
	{
		cout << "Working!" << endl;
	}
	
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
	glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0,0,width, height);

	// Render as normal here
	// output goes to the FBO and it’s attached buffers
	LookAt();
	DrawCube();
	
	float depths[width*height];
	//glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	//glReadBuffer(GL_DEPTH_COMPONENT24);
	//glReadBuffer(GL_RENDERBUFFER_EXT);
	//glReadBuffer(GL_FRAMEBUFFER_EXT);
	//glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, depths); //(x,y, width, height, format, type, *pixels)
	//WriteDepthFileVIL("buffer.jpg", depths);
	WriteDepthFileVIL("buffer.jpg", depthbuffer);

	glPopAttrib();
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	//clean up
	glDeleteFramebuffersEXT(1, &fbo);
	glDeleteRenderbuffersEXT(1, &depthbuffer);

	return 0;
}

void CheckErrors()
{
	GLenum error = glGetError();
	cout << gluErrorString(error) << endl;
}

void LookAt()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(70, 1, 1, 100);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	//gluLookAt(2, 2, 10, 2, 0, 0, 0, 1, 0);
	gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0);
	
}
/*
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
*/