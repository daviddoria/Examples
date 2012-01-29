/* mail to oguz_oz_han@hotmail.com if any questions */
#include <glut.h>
 
#include <stdio.h>
#include <stdlib.h>
 
#define SW 400
#define SH 400
 
 int selected = 0;
 
 void gl_draw();
 void gl_init(int w, int h);
 void gl_select(int x, int y);
 void gl_selall(GLint hits, GLuint *buff);
 void mouseClick();
 void mousedw(int x, int y, int but);
 void draw_block(float x, float y, float z);
 void list_hits(GLint hits, GLuint *names);
 
#include <iostream>
			 
using namespace std;
 
 void gl_draw()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
 
	glTranslatef(0.0, 0.0, -5.0);
 
	glColor3f(1.0, 0.0, 0.0);
	glLoadName(7); /* Overwrite the first name in the buffer */
	draw_block(-0.3, 0, -2);
 
	glColor3f(0.0, 1.0, 0.0);
	glLoadName(14); /* Overwrite the first name in the buffer */
	draw_block(0, 0, -4);
 
	glColor3f(0.0, 0.0, 1.0);
	glLoadName(21); /* Overwrite the first name in the buffer */
	draw_block(0.3, 0, -6);
 
	glutSwapBuffers();
}
 
 void gl_init(int w, int h)
{
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glViewport(0, 0, w, h);
 
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
 
	gluPerspective(60.0, 1.0, 0.0001, 1000.0);
 
	glMatrixMode(GL_MODELVIEW);
}
 
 void mouseClick(int button, int state, int x, int y)
{
	if 	((button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN))
	{
		mousedw(x, y, button);
	}
}
 
 void mousedw(int x, int y, int but)
{
	//printf("Mouse button %d pressed at %d %d\n", but, x, y);
	cout << x << " " << y << endl;
	gl_select(x,SH-y); //Important: gl (0,0) ist bottom left but window coords (0,0) are top left so we have to change this!
}
 
 void gl_select(int x, int y)
{
	GLuint buff[64] = {0};
	GLint hits, view[4];
	int id;
 
 	/*
	This choose the buffer where store the values for the selection data
	*/
	glSelectBuffer(64, buff);
 
 	/*
	This retrieve info about the viewport
	*/
	glGetIntegerv(GL_VIEWPORT, view);
 
 	/*
	Switching in selecton mode
	*/
	glRenderMode(GL_SELECT);
 
 	/*
	Clearing the name's stack
	This stack contains all the info about the objects
	*/
	glInitNames();
 
 	/*
	Now fill the stack with one element (or glLoadName will generate an error)
	*/
	glPushName(0);
 
 	/*
	Now modify the vieving volume, restricting selection area around the cursor
	*/
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
 
 		/*
	restrict the draw to an area around the cursor
		*/
	gluPickMatrix(x, y, 1.0, 1.0, view);
	gluPerspective(60, 1.0, 0.0001, 1000.0);
 
 		/*
	Draw the objects onto the screen
		*/
	glMatrixMode(GL_MODELVIEW);
 
 		/*
	draw only the names in the stack, and fill the array
		*/
	glutSwapBuffers();
	gl_draw();
 
 		/*
	Do you remeber? We do pushMatrix in PROJECTION mode
		*/
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
 
 	/*
	get number of objects drawed in that area
	and return to render mode
	*/
	hits = glRenderMode(GL_RENDER);
 
 	/*
	Print a list of the objects
	*/
	list_hits(hits, buff);
 
 	/*
	uncomment this to show the whole buffer
 	* /
	gl_selall(hits, buff);
	*/
 
	glMatrixMode(GL_MODELVIEW);
}
 
 void gl_selall(GLint hits, GLuint *buff)
{
	GLuint *p;
	int i;
 
	gl_draw();
 
	p = buff;
	for (i = 0; i < 6 * 4; i++)
	{
		printf("Slot %d: - Value: %d\n", i, p[i]);
	}
 
	printf("Buff size: %x\n", (GLbyte)buff[0]);
}
 
 void draw_block(float x, float y, float z)
{
	glPushMatrix();
	glTranslatef(x, y, z);
	glutSolidCube(1.0);
	glPopMatrix();
}
 
 void list_hits(GLint hits, GLuint *names)
{
	int i;
 
 	/*
 		For each hit in the buffer are allocated 4 bytes:
	1. Number of hits selected (always one,
	beacuse when we draw each object
	we use glLoadName, so we replace the
	prevous name in the stack)
	2. Min Z
	3. Max Z
	4. Name of the hit (glLoadName)
	*/
 
	printf("%d hits:\n", hits);
 
	for (i = 0; i < hits; i++)
		printf(	"Number: %d\n"
				"Min Z: %d\n"
				"Max Z: %d\n"
				"Name on stack: %d\n",
    (GLubyte)names[i * 4],
     (GLubyte)names[i * 4 + 1],
      (GLubyte)names[i * 4 + 2],
       (GLubyte)names[i * 4 + 3]
		      );
 
	printf("\n");
}
 
 int main(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB|GLUT_DOUBLE);
	glutInitWindowSize(SW, SH);
	glutInitWindowPosition(0, 0);
	glutCreateWindow(argv[0]);
	glutDisplayFunc(gl_draw);
	glutMouseFunc(mouseClick);
 
	gl_init(SW, SH);
 
	glutMainLoop();
 
	return EXIT_SUCCESS;
}