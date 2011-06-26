#include "itkImage.h"
#include "itkImageFileWriter.h"

#include <iostream>

int main(int, char *[])
{
	typedef itk::Image< bool, 3 > ImageType;
	ImageType::Pointer image = ImageType::New();
	ImageType::IndexType start;

	start[0] =   0;  // first index on X
	start[1] =   0;  // first index on Y
	start[2] =   0;  // first index on Z
	
	ImageType::SizeType  size;

	size[0]  = 200;  // size along X
	size[1]  = 200;  // size along Y
	size[2]  = 200;  // size along Z
	
	ImageType::RegionType region;
  
	region.SetSize( size );
	region.SetIndex( start );
	
	image->SetRegions( region );
	image->Allocate();
	
	ImageType::IndexType pixelIndex;
 
	pixelIndex[0] = 27;   // x position
	pixelIndex[1] = 29;   // y position
	pixelIndex[2] = 37;   // z position

	image->SetPixel(   pixelIndex,   true  );
	
	typedef itk::ImageFileWriter< ImageType > WriterType;
	WriterType::Pointer writer = WriterType::New();

	//writer->SetFileName( "test.vti" );
	writer->SetFileName( "test.vtp" );

	writer->SetInput(  image  );

	try
	{
		writer->Update();
	}
	catch( itk::ExceptionObject & exp ) 
	{
		std::cerr << "Exception caught !" << std::endl;
		std::cerr << exp << std::endl;
	}
	return 0;
}