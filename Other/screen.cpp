#include <iostream>
#include <X11/Xlib.h>

using namespace std;

static void captureFrame (int num, int width, int height)
{
  fprintf (stderr,"capturing frame %04d\n",num);
	

  char s[100];
  sprintf (s,"frame/frame%04d.ppm",num);

	display = XOpenDisplay(DEFAULT_DISPLAY);

	FILE *f = fopen (s,"wb");
//  if (!f) 
	//dsError ("can't open \"%s\" for writing",s);
  fprintf (f,"P6\n%d %d\n255\n",width,height);
  XImage *image = XGetImage (display,win,0,0,width,height,~0,ZPixmap);
  int rshift = 7 - getHighBitIndex (image->red_mask);
  int gshift = 7 - getHighBitIndex (image->green_mask);
  int bshift = 7 - getHighBitIndex (image->blue_mask);

for (int y=0; y<height; y++)
{
    for (int x=0; x<width; x++)
	{
      unsigned long pixel = XGetPixel (image,x,y);
      unsigned char b[3];
      b[0] = SHIFTL(pixel & image->red_mask,rshift);
      b[1] = SHIFTL(pixel & image->green_mask,gshift);
      b[2] = SHIFTL(pixel & image->blue_mask,bshift);
      fwrite (b,3,1,f);
    }
  }


  fclose (f);
//  XDestroyImage (image);
}

int main()
{
	return 0;
}
