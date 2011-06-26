#include <glut.h>
#include "Display.h"
#include <iostream>

using namespace std;

class WindowClass
{
	private:	
		static double AxisLength;
		
		static void ClassDisplay()
		{
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			
			//double AxisLength = 3;
			DrawLine(geom_Point3(0,0,0), geom_Point3(AxisLength,0,0), 5 , 'r');
			DrawLine(geom_Point3(0,0,0), geom_Point3(0,AxisLength,0), 5, 'g');
			DrawLine(geom_Point3(0,0,0), geom_Point3(0,0,AxisLength), 5, 'b');
			
//			DrawAxes();
			glutSwapBuffers();
		}
		//void ClassKeyboard();
public:
		WindowClass(int argc, char *argv[])
		{
			int WindowHeight = 1000;
			int WindowWidth = 1000;

			AxisLength = 3;
	
			glutInit(&argc, argv);
	
			glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
			glutInitWindowSize(WindowWidth,WindowHeight);
			glutInitWindowPosition(0,0);
			glutCreateWindow("Test");
			glutDisplayFunc(ClassDisplay);
			//glutKeyboardFunc(ClassKeyboard);
		}	
			
};
