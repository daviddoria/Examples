#include "itkImage.h"
#include "itkImageFileWriter.h"

int main( int argc, char *argv[])
{
	typedef unsigned char     PixelType;
	const     unsigned int    Dimension = 2;
	typedef itk::Image< PixelType, Dimension >  ImageType;

	ImageType::RegionType region;
	ImageType::IndexType start;
	start[0] = 0;
	start[1] = 0;

	std::cout << start << std::endl;
	
	start[0] += 3;
	std::cout << start << std::endl;
	
	return 0;
}