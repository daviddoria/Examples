#include <iostream>

#include <GL/glut.h>

#include <vil/vil_rgb.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>
#include <vil/vil_image_view.h>
#include <vil/vil_convert.h>

#include <vbl/vbl_array_3d.h>

using namespace std;

void Screenshot(const string &filename)
{
  const unsigned int Width = glutGet(GLUT_WINDOW_WIDTH);
  const unsigned int Height = glutGet(GLUT_WINDOW_HEIGHT);

  vil_image_view<vil_rgb<vxl_byte> > RGBImage(Width, Height, 1, 1);

  vbl_array_3d<GLubyte> buffer(Height,Width,3);

  glReadBuffer(GL_FRONT);

  glReadPixels(0, 0, Width, Height, GL_RGB, GL_UNSIGNED_BYTE, buffer.begin());

  GLubyte r,g,b;
  for(unsigned int col = 0; col < Width; col++)
  {
      for(unsigned int row = 0; row < Height; row++)
      {
          //works, but upside down
// 			r = buffer(row, col, 0);
// 			g = buffer(row, col, 1);
// 			b = buffer(row, col, 2);
          r = buffer(Height - row - 1, col, 0);
          g = buffer(Height - row - 1, col, 1);
          b = buffer(Height - row - 1, col, 2);
          RGBImage(col, row) = vil_rgb<vxl_byte>(r,g,b);
      }
  }

  vil_save(RGBImage, filename.c_str());

  cout << "Done writing RGB image." << endl;
}

void ScreenshotFixed(const string &filename)
{
  unsigned int Width = 200;
  unsigned int Height = 100;

  GLubyte buffer[100][200][3];
  glReadPixels(0, 0, Width, Height, GL_RGB, GL_UNSIGNED_BYTE, buffer);

  vil_image_view<vil_rgb<vxl_byte> > RGBImage(Width, Height, 1, 1); //(cols, rows, 1, 1)

  GLubyte r,g,b;

  for(unsigned int i = 0; i < Width; ++i)
  {
      for(unsigned int j = 0; j < Height; ++j)
      {
          r = buffer[j][i][0];
          g = buffer[j][i][1];
          b = buffer[j][i][2];
          RGBImage(i, j) = vil_rgb<vxl_byte>(r,g,b);
      }
  }

  vil_save(RGBImage, filename.c_str());

  cout << "Done writing RGB image." << endl;
}