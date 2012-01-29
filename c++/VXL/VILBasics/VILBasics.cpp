#include <iostream>
#include <cstdio> //drand


#include <vil/vil_rgb.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>
#include <vil/vil_image_view.h>
#include <vil/vil_convert.h>

using namespace std;

void WriteGreyscaleImage(void);
void WriteColorImage(void);

void ReadGreyscaleImage(void);
void ReadColorImage(void);


int main(int argc, char **argv)
{
	cout << "VIL Basics" << "--------------" << endl;
	
	WriteGreyscaleImage();
	ReadGreyscaleImage();
	
	//WriteColorImage();
	//ReadColorImage();
	
	return 0;
}

void WriteGreyscaleImage(void)
{
	//a 1 plane, grey scale image
	vil_image_view<vxl_byte> BWimg(100,100,1,1); //(ni, nj, n_planes, n_interleaved_planes)
	
	for (unsigned j = 0; j < BWimg.nj(); ++j)
	{
		for (unsigned i = 0; i < BWimg.ni(); ++i)
		{
			//img(i,j) = vil_rgb<vxl_byte>(drand48(),0,0);
			BWimg(i,j) = 255. * drand48();
		}
	}
	
	vil_save(BWimg, "GreyTestWrite.jpg");

}

void WriteColorImage(void)
{
	//a 1 plane, RGB image
	vil_image_view<vil_rgb<vxl_byte> > ColorImg(100,100,1,1); //(ni, nj, n_planes, n_interleaved_planes)
	
	for (unsigned j = 0; j < ColorImg.nj(); ++j)
	{
		for (unsigned i = 0; i < ColorImg.ni(); ++i)
		{
			ColorImg(i,j) = vil_rgb<vxl_byte>(255.*drand48(), 255.*drand48(), 255.*drand48());
		}
	}
	
	vil_save(ColorImg, "ColorTest.jpg");

}


void ReadGreyscaleImage(void)
{
	//read a file
	vil_image_view<vxl_byte> Image = vil_load("GreyTestWrite.jpg");
	
	//write it again to make sure it was read correctly
	vil_save(Image, "GreyTestReadWrite.jpg");
}

void ReadColorImage(void)
{
	vil_image_view<vil_rgb<vxl_byte> > ColorImage = vil_load("ColorTestWrite.jpg");
	vil_save(ColorImage, "ColorTestReadWrite.jpg");
}
