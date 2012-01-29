#include "OpenGLinit.h"

//void OpenGLinit(string WindowTitle)
void OpenGLinit(int argc, char *argv[], string WindowTitle)
{
	int WindowHeight = 1000;
	int WindowWidth = 1000;

	//Initialise GLUT and create a window
	glutInit(&argc, argv);
	//glutInit(NULL, NULL);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(WindowWidth,WindowHeight);
	glutInitWindowPosition(0,0);
	glutCreateWindow(WindowTitle.c_str());
	

	glScalef(0.25,0.25,0.25);
	
	zprInit();

	// Initialise OpenGL
	
	//glDisable(GL_LIGHTING);
	glEnable(GL_LIGHTING);
	GLfloat global_ambient[] = {0.1, 0.1, 0.1, 0.1};
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);
	
	//glShadeModel(GL_FLAT);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHT0);
	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	glEnable(GL_COLOR_MATERIAL);
	
	//back face culling
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
}
