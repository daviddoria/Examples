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

	ImageType::SizeType size;
	size[0] = 200;
	size[1] = 300;

	region.SetSize(	size);
	region.SetIndex(start);

	ImageType::Pointer image = ImageType::New();
	image->SetRegions(region);
	image->Allocate();

	ImageType::IndexType pixelIndex;
	pixelIndex[0] = 100;
	pixelIndex[1] = 100;
	
	//ImageType::PixelType pixelValue = image->GetPixel(pixelIndex);

	image->SetPixel(pixelIndex, 100);

	typedef  itk::ImageFileWriter< ImageType  > WriterType;
	WriterType::Pointer writer = WriterType::New();
	writer->SetFileName( "test.png" );
	writer->SetInput(image);	
	writer->Update();

  return 0;
}