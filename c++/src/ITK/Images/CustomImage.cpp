#include "itkImage.h"
#include "CustomClass.h"

#include <iostream>

int main( int argc, char *argv[])
{
	typedef CustomClass PixelType;
	const unsigned int Dimension = 2;
	typedef itk::Image< PixelType, Dimension > ImageType;
	
	//the "corner" of the image should be at (0,0)
	ImageType::IndexType start;
	start[0] = 0;
	start[1] = 0;

	//create a 200x300 image
	ImageType::SizeType size;
	size[0] = 200;
	size[1] = 300;

	//setup the image
	ImageType::RegionType region;
	region.SetSize(size);
	region.SetIndex(start);

	//create an image
	ImageType::Pointer image = ImageType::New();
	image->SetRegions(region);
	image->Allocate();
	
	//set the (10,10) pixel value
	ImageType::IndexType ind;
	ind[0] = 10;
	ind[1] = 10;
	
	CustomClass CC;
	CC.Double = 23.1;
	CC.UnsignedInt = 5;
	
	image->SetPixel(ind, CC);
			
	ImageType::PixelType pixValue = image->GetPixel(ind);
	std::cout << pixValue.Double << " " << pixValue.UnsignedInt << std::endl;
	
	return 0;
}