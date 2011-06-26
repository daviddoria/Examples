#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkImageFileReader.h"
#include "itkRawImageIO.h"

void Write();
void Read();

typedef double     PixelType;
const     unsigned int    Dimension = 2;
typedef itk::Image< PixelType, Dimension >  ImageType;

int main( int argc, char *argv[])
{
	Write();
	Read();
	
	return 0;
}

void Write()
{
	ImageType::RegionType region;
	ImageType::IndexType start;
	start[0] = 0;
	start[1] = 0;

	ImageType::SizeType size;
	size[0] = 200;
	size[1] = 300;

	region.SetSize(size);
	region.SetIndex(start);

	ImageType::Pointer image = ImageType::New();
	image->SetRegions(region);
	image->Allocate();

	ImageType::IndexType pixelIndex;
	pixelIndex[0] = 100;
	pixelIndex[1] = 100;
	
	PixelType val = 5.0;
	
	image->SetPixel(pixelIndex, val);
		
	itk::RawImageIO<PixelType,2>::Pointer io;
	io = itk::RawImageIO<PixelType,2>::New();
	
	typedef  itk::ImageFileWriter< ImageType  > WriterType;
	WriterType::Pointer writer = WriterType::New();
	writer->SetFileName( "test.raw" );
	writer->SetInput(image);	
	writer->SetImageIO(io);
	
	try
	{
		writer->Update();
	}
	catch( itk::ExceptionObject & excp )
	{
		std::cerr << "Error while writing the image " << std::endl;
		std::cerr << excp << std::endl;
	}
}


void Read()
{

	itk::RawImageIO<PixelType,2>::Pointer io;
	io = itk::RawImageIO<PixelType,2>::New();
	io->SetFileDimensionality(Dimension);
	io->SetNumberOfDimensions(Dimension);
	
	unsigned int dim[Dimension] = {200,300};
	
	for(unsigned int i=0; i<Dimension; i++)
	{
		io->SetDimensions(i,dim[i]);
	}
	
	itk::ImageFileReader<ImageType>::Pointer reader;
	reader = itk::ImageFileReader<ImageType>::New();
	reader->SetImageIO(io);
	reader->SetFileName("test.raw");
	
	try
	{
		reader->Update();
	}
	catch( itk::ExceptionObject & excp )
	{
		std::cerr << "Error while reading the image " << std::endl;
		std::cerr << excp << std::endl;
	}
	
	ImageType::Pointer image = reader->GetOutput();
	
	ImageType::IndexType pixelIndex;
	pixelIndex[0] = 100;
	pixelIndex[1] = 100;
	
	PixelType val = image->GetPixel(pixelIndex);
		
	std::cout << "val: " << val << std::endl;
	
	
}