#include "Display.h"

void DrawPoint(const geom_Point3 &P, const double rad, const char Color)
{
	glPushMatrix();
		SetColor(Color);

		glTranslatef(P.getX(), P.getY(), P.getZ());
		glutSolidSphere(rad, 20, 20);
	glPopMatrix();
}

void DrawLine(const geom_Point3 &P1, const geom_Point3 &P2, const double width, const char Color)
{
	if(width == 0)
		glLineWidth(DEFAULTWIDTH);
	else
		glLineWidth(width);
	
	SetColor(Color);
	glBegin(GL_LINES);
		glVertex3d(P1.getX(), P1.getY(), P1.getZ());
		glVertex3d(P2.getX(), P2.getY(), P2.getZ());
	glEnd();
}

void DrawVector(const geom_Vector3 &V, const double length, const double width, const char Color)
{
	geom_Point3 P1(0,0,0);
	geom_Point3 P2 = P1 + V.Normalized() * length;
	DrawLine(P1, P2, width, Color);
}

void DrawRay(const geom_Ray3 &R, const double length, const double width, const char Color)
{
	geom_Point3 P1 = R.getOrigin();
	geom_Point3 P2 = P1 + length*R.getDirection();
	DrawLine(P1, P2, width, Color);
}

void DrawTriangle(const geom_Point3 &P1, const geom_Point3 &P2, const geom_Point3 &P3, const double width, const char Color)
{
	if(width == 0)
		glLineWidth(DEFAULTWIDTH);
	else
		glLineWidth(width);
	
	SetColor(Color);
	
	//double Nx, Ny, Nz;
	//Nx = 
	glBegin(GL_TRIANGLES);
		//glNormal3d
		glVertex3d(P1.getX(), P1.getY(), P1.getZ());
		glVertex3d(P2.getX(), P2.getY(), P2.getZ());
		glVertex3d(P3.getX(), P3.getY(), P3.getZ());
	glEnd();
}

void DrawTriangle(const geom_Triangle3 &T, const double width, const char Color)
{
	if(width == 0)
		glLineWidth(DEFAULTWIDTH);
	else
		glLineWidth(width);
	
	SetColor(Color);
	
	geom_Point3 P1 = T.getP1();
	geom_Point3 P2 = T.getP2();
	geom_Point3 P3 = T.getP3();
	
	double Nx, Ny, Nz;
	geom_Vector3 N = T.getNormal();
	Nx = N.getX();
	Ny = N.getY();
	Nz = N.getZ();
	//DrawTriangle(P1, P2, P3, width, Color);
	
	glBegin(GL_TRIANGLES);
		glNormal3d(Nx, Ny, Nz);
		glVertex3d(P1.getX(), P1.getY(), P1.getZ());
		//glNormal3d(Nx, Ny, Nz);
		glVertex3d(P2.getX(), P2.getY(), P2.getZ());
		//glNormal3d(Nx, Ny, Nz);
		glVertex3d(P3.getX(), P3.getY(), P3.getZ());
	glEnd();
}

void SetColor(const char Color)
{
	if(Color == 'w')
		glColor3f(1,1,1); //white
	else if(Color == 'r')
		glColor3f(1,0,0); //red
	else if(Color == 'g')
		glColor3f(0,1,0); //green
	else if(Color == 'b')
		glColor3f(0,0,1); //blue
}

void DrawAxes(const double AxisLength)
{
	//x axis
	DrawLine(geom_Point3(0,0,0), geom_Point3(AxisLength,0,0), 5 , 'r');
	
	//y axis
	DrawLine(geom_Point3(0,0,0), geom_Point3(0,AxisLength,0), 5, 'g');

	//z axis
	DrawLine(geom_Point3(0,0,0), geom_Point3(0,0,AxisLength), 5, 'b');	
}



void DrawGrid(const string WhichGrid, const int Size, const double width, const char Color)
{
	int x,y,z;
	if( (WhichGrid == "XY") || (WhichGrid == "YX") )
	{
		for(x = -Size/2; x <= Size/2; x++)
		{
			y = Size/2;
			geom_Point3 P1(x,y,0);
			geom_Point3 P2(x,-y,0);
			DrawLine(P1, P2, width, Color);
		}
	
		for(y = -Size/2; y <= Size/2; y++)
		{
			int x = Size/2;
			geom_Point3 P1(x, y, 0);
			geom_Point3 P2(-x, y, 0);
			DrawLine(P1, P2, width, Color);
		}
	}
	else if( (WhichGrid == "YZ") || (WhichGrid == "ZY") )
	{
		for(z = -Size/2; z <= Size/2; z++)
		{
			y = Size/2;
			geom_Point3 P1(0,y,z);
			geom_Point3 P2(0,-y,z);
			DrawLine(P1, P2, width, Color);
		}
	
		for(y = -Size/2; y <= Size/2; y++)
		{
			x = Size/2;
			geom_Point3 P1(0, y, z);
			geom_Point3 P2(0, y, -z);
			DrawLine(P1, P2, width, Color);
		}	
	}
	else if( (WhichGrid == "ZX") || (WhichGrid == "XZ") )
	{
		for(x = -Size/2; x <= Size/2; x++)
		{
			z = Size/2;
			geom_Point3 P1(x,0,z);
			geom_Point3 P2(x,0,-z);
			DrawLine(P1, P2, width, Color);
		}
	
		for(z = -Size/2; z <= Size/2; z++)
		{
			x = Size/2;
			geom_Point3 P1(x, 0, z);
			geom_Point3 P2(-x, 0, z);
			DrawLine(P1, P2, width, Color);
		}	
	}
}

void WriteText(float x, float y, float z, string Text) 
{  
	glPushMatrix();
	glTranslatef(x, y,z);
	for (int i = 0; i < Text.size(); i++)
	{
		glutStrokeCharacter(GLUT_STROKE_ROMAN, Text[i]);
	}
	glPopMatrix();
}

void DrawCube()
{
	glBegin(GL_QUADS);
		glColor3f(1.0f,0.0f,0.0f); // Red
		glVertex3f(-1.0f, 1.0f, 1.0f);
		glVertex3f( 1.0f, 1.0f, 1.0f);
		glVertex3f( 1.0f, 1.0f,-1.0f);
		glVertex3f(-1.0f, 1.0f,-1.0f);
	
		glColor3f(0.0f,0.0f,1.0f); // Blue
		glVertex3f(-1.0f,-1.0f, 1.0f);
		glVertex3f(-1.0f,-1.0f,-1.0f);
		glVertex3f( 1.0f,-1.0f,-1.0f);
		glVertex3f( 1.0f,-1.0f, 1.0f);
	glEnd();
     glBegin(GL_TRIANGLE_STRIP);
	glColor3f(1.0f,1.0f,1.0f); // White
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f,-1.0f, 1.0f);
	glVertex3f( 1.0f, 1.0f, 1.0f);
	glColor3f(0.0f,1.0f,0.0f); // Green
	glVertex3f( 1.0f,-1.0f, 1.0f);
	glColor3f(1.0f,1.0f,0.0f); // Yellow
	glVertex3f( 1.0f, 1.0f,-1.0f);
	glColor3f(0.0f,1.0f,1.0f); // Aqua?
	glVertex3f( 1.0f,-1.0f,-1.0f);
	glColor3f(0.6f,0.6f,0.6f); // Gray
	glVertex3f(-1.0f, 1.0f,-1.0f);
	glColor3f(0.1f,0.1f,0.1f); // Dark Gray
	glVertex3f(-1.0f,-1.0f,-1.0f);
	glColor3f(0.0f,0.0f,1.0f); // Blue
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glColor3f(1.0f,0.0f,1.0f); // Red
	glVertex3f(-1.0f,-1.0f, 1.0f);
     glEnd();
}