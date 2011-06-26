#include "itkImage.h"
#include "itkImageFileWriter.h"

#include <iostream>

int main(int argc, char *argv[])
{
	
	typedef itk::Image< float, 3 > ImageType;
	ImageType::Pointer image = ImageType::New();
	
	ImageType::IndexType start;
	start[0] = 0;  // first index on X
	start[1] = 0;  // first index on Y
	start[2] = 0;  // first index on Z
	
	ImageType::SizeType  size;
	size[0] = 20;  // size along X
	size[1] = 20;  // size along Y
	size[2] = 20;  // size along Z
	
	ImageType::RegionType region;
	region.SetSize( size );
	region.SetIndex( start );
	image->SetRegions( region );
	
	
	//breaks it
	/*
	ImageType::SpacingType spacing;
	spacing[0] = .5;
	spacing[1] = .5;
	spacing[2] = .5;
	image->SetSpacing(spacing);
	*/
	
	/*
	//does not break it
	ImageType::PointType origin;
	origin[0] = -5.0;
	origin[1] = -5.0;
	origin[2] = -5.0;
	image->SetOrigin(origin);
	*/
	
	image->Allocate();
	
	
	//does not break it
	ImageType::SpacingType spacing;
	spacing[0] = .5;
	spacing[1] = .5;
	spacing[2] = .5;
	image->SetSpacing(spacing);
	
	
	ImageType::IndexType pixelIndex;
	pixelIndex[0] = 7;   // x position
	pixelIndex[1] = 9;   // y position
	pixelIndex[2] = 7;   // z position
	image->SetPixel( pixelIndex, 10.0 );
	
	ImageType::IndexType pixelIndex0;
	pixelIndex0[0] = 0;   // x position
	pixelIndex0[1] = 0;   // y position
	pixelIndex0[2] = 0;   // z position
	image->SetPixel( pixelIndex0, 10.0 );
	
	/*
	//set the nearest voxel to a point to 4.0
	ImageType::PointType point;
	point[0] = 1.4;
	point[1] = 2.3;
	point[2] = 3.2;
	
	ImageType::IndexType ClosestPixel;
	
	bool isInside = image->TransformPhysicalPointToIndex(point, ClosestPixel);
	if(isInside)
	{
	image->SetPixel( ClosestPixel, 4.0 );
	std::cout << "The closest pixel was at index: " << ClosestPixel << std::endl;
}
	else
	{
	std::cout << "Requested point is not inside the image!" << std::endl;
}
	*/
	
	typedef itk::ImageFileWriter< ImageType > WriterType;
	WriterType::Pointer writer = WriterType::New();

	writer->SetFileName( "test.vtk" );

	writer->SetInput(  image  );
	writer->Update();
	
	return 0;
}