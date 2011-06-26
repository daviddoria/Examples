#include "itkImageRegion.h"
#include "itkIndex.h"
#include "itkSize.h"

void Cropping();
void Basics();

int main(int, char *[])
{
  //Basics();
  Cropping();
  return EXIT_SUCCESS;
}

void Basics()
{
  itk::Size<2> size;
  size.Fill(3);

  itk::Index<2> index;
  index.Fill(1);

  typedef itk::ImageRegion<2> RegionType;
  RegionType region(index,size);

  std::cout << region << std::endl;
}

void Cropping()
{
  typedef itk::ImageRegion<2> RegionType;

  // Setup big region
  itk::Size<2> bigSize;
  bigSize.Fill(10);

  itk::Index<2> bigIndex;
  bigIndex.Fill(0);

  RegionType bigRegion(bigIndex,bigSize);
  std::cout << bigRegion << std::endl;

  // Setup little region
  itk::Size<2> littleSize;
  littleSize.Fill(5);

  itk::Index<2> littleIndex;
  littleIndex.Fill(8);

  RegionType littleRegion(littleIndex,littleSize);

  std::cout << littleRegion << std::endl;

  RegionType bigCroppedWithLittle = bigRegion;
  bigCroppedWithLittle.Crop(littleRegion);
  std::cout << bigCroppedWithLittle << std::endl;

  RegionType littleCroppedWithBig = littleRegion;
  littleCroppedWithBig.Crop(bigRegion);
  std::cout << littleCroppedWithBig << std::endl;
}
