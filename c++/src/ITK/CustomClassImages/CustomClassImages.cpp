#include "itkImage.h"
#include "itkImageFileWriter.h"

#include "TestClass.h"

int main( int argc, char *argv[])
{
  typedef TestClass     PixelType;
  const     unsigned int    Dimension = 2;
  typedef itk::Image< PixelType, Dimension >  ImageType;

  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size[0] = 200;
  size[1] = 300;

  ImageType::RegionType region(start, size);

  ImageType::Pointer image = ImageType::New();
  image->SetRegions(region);
  image->Allocate();

  ImageType::IndexType pixelIndex;
  pixelIndex.Fill(100);

  TestClass MyClass;
  MyClass.MyDouble = 3.0;
  MyClass.MyString = "hello";

  image->SetPixel(pixelIndex, MyClass);

  ImageType::PixelType pixelValue = image->GetPixel(pixelIndex);

  std::cout << pixelValue.MyDouble << " " << pixelValue.MyString << std::endl;

  return EXIT_SUCCESS;
}