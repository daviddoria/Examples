#include <iostream>
#include <vector>
#include <map>
#include <cstdio>

#include <glut.h>

void display();
void LookAt();
void DrawCube(const bool PickColors);

#define PICK_BUFFER_SIZE 256
unsigned int PickBuffer[PICK_BUFFER_SIZE];

void Pick(GLdouble x, GLdouble y);

void MouseButtonsPick(int button, int state, int x, int y);
void polygon(int a, int b, int c , int d);

void Array2Vec(unsigned char arr[3], std::vector<unsigned char> &out);
void OutputVector(std::vector<unsigned char> &arr);

int WindowHeight = 1000;
int WindowWidth = 1000;
	

//vector<vector<unsigned char> > ColorList(256*256*256);
std::map<std::vector<unsigned char>, int> ColorMap;
std::vector<std::vector<unsigned char> > ColorList;
void GenerateColors(std::map<std::vector<unsigned char>, int> &Map, std::vector<std::vector<unsigned char> > &List, int num);

int main(int argc, char *argv[])
{
  GenerateColors(ColorMap, ColorList, 10);
  
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(WindowWidth, WindowHeight);
  glutInitWindowPosition(0, 0);
  
  glutCreateWindow("Picking Example");
  
  glutDisplayFunc(display);
  glutMouseFunc(MouseButtonsPick);

  glEnable(GL_DEPTH_TEST);
  
  LookAt();
  glutMainLoop();
  return 0;
}

void GenerateColors(std::map<std::vector<unsigned char>, int> &Map, std::vector<std::vector<unsigned char> > &List, int num)
{
  std::vector<unsigned char> junk(3,-1);
  List = std::vector<std::vector<unsigned char> > (num + 1, junk);
  int ColorIndex = 1;
	  
  for(unsigned char r = 1; r <= 255 && r > 0; r++) // or (int)r <= 255
  {
    //printf("r: %u\n", r);
    for(unsigned char g = 1; g <= 255 && g > 0; g++)
    {
      //printf("g: %u\n", g);
      for(unsigned char b = 1; b <= 255 && b>0; b++)
      {
	//printf("b: %u\n", b);
	std::vector<unsigned char> color(3,0);
	color[0] = r;
	color[1] = g;
	color[2] = b;
	
	Map[color] = ColorIndex;
	List[ColorIndex] = color;
	//ColorList.push_back(color);
			
	ColorIndex++;
	if(ColorIndex > num)
	{
	  return;
	}
      }
    }
  }
	
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
  DrawCube(false);
  
  glutSwapBuffers();
}


GLfloat vertices[][3] = {{-1.0,-1.0,-1.0},{1.0,-1.0,-1.0},
		{1.0,1.0,-1.0}, {-1.0,1.0,-1.0}, {-1.0,-1.0,1.0}, 
  		{1.0,-1.0,1.0}, {1.0,1.0,1.0}, {-1.0,1.0,1.0}};

void DrawCube(const bool PickColors)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if(PickColors)
  {
    glDisable(GL_DITHER);
    glDisable(GL_LIGHTING);
  }
  else
  {
    glEnable(GL_DITHER);
    //glEnable(GL_LIGHTING);
  }

  if(!PickColors)
  {
    glColor3ub(255, 0, 0);	// Color Red
  }
  else
  {
    glColor3ub(1, 1, 1);
  }

  polygon(0,3,2,1);

  if(!PickColors)
  {
    glColor3ub(0,255,0);	// Color Green
  }
  else
  {
    glColor3ub(1, 1, 2);
  }

  polygon(2,3,7,6);

  if(!PickColors)	
  {
    glColor3ub(0, 0, 255);	// Color Blue
  }
  else
  {
    glColor3ub(1, 1, 3);
  }

  polygon(0,4,7,3);

  if(!PickColors)
  {
    glColor3ub(255,255,0);	// Color Yellow
  }
  else
  {
    glColor3ub(1, 1, 4);
  }

  polygon(1,2,6,5);

  if(!PickColors)
  {
    glColor3ub(0,255,255);	// Color Cyan 
  }
  else
  {
    glColor3ub(1, 1, 5);
  }

  polygon(4,5,6,7);

  if(!PickColors)
  {
    glColor3ub(255,0,255);	// Color Magenta
  }
  else
  {
    glColor3ub(1, 1, 6);
  }

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
    double newy = glutGet(GLUT_WINDOW_HEIGHT) - 1 - y;
    Pick(x, newy);
  }
}



void Pick(GLdouble x, GLdouble y)
{
  DrawCube(true);
  //glutSwapBuffers();

  unsigned char pixelub[3];
  glReadPixels(x, y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, pixelub);
  
  printf("r: %u g: %u b: %u\n", pixelub[0], pixelub[1], pixelub[2]);
  
  std::vector<unsigned char> v(3,0);
  Array2Vec(pixelub, v);
  
  printf("r: %u g: %u b: %u\n", v[0], v[1], v[2]);
  
  int c = ColorMap[v];
  std::cout << "Color Index: " << c << std::endl;
  std::cout << "Corresponding Color: ";
  OutputVector(ColorList[c]);
  std::cout << std::endl;
}

void Array2Vec(unsigned char arr[3], std::vector<unsigned char> &out)
{
  for(int i = 0; i <= 2; i++)
  {
    out[i] = arr[i];
  }
	
}

void OutputVector(std::vector<unsigned char> &arr)
{
  for(int i = 0; i <= 2; i++)
  {
    std::cout << (int)arr[i] << " " ;
  }
  
  std::cout << std::endl;
	
}
