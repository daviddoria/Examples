#include <itkImage.h>

int main(int, char*[])
{
  typedef itk::Image<unsigned char, 2> ImageType;

  // Big region
  ImageType::RegionType bigRegion;

  ImageType::SizeType bigSize;
  bigSize[0] = 100;
  bigSize[1] = 100;

  ImageType::IndexType bigStart;
  bigStart[0] = 0;
  bigStart[1] = 0;
  
  bigRegion.SetSize(bigSize);
  bigRegion.SetIndex(bigStart);

  // Small inside region
  ImageType::RegionType smallInsideRegion;

  ImageType::SizeType smallInsideSize;
  smallInsideSize[0] = 10;
  smallInsideSize[1] = 10;

  ImageType::IndexType smallInsideStart;
  smallInsideStart[0] = 50;
  smallInsideStart[1] = 50;

  smallInsideRegion.SetSize(smallInsideSize);
  smallInsideRegion.SetIndex(smallInsideStart);

  std::cout << "Small inside region is " << bigRegion.IsInside(smallInsideRegion) << std::endl;
  
  // Small outside region
  ImageType::RegionType smallOutsideRegion;

  ImageType::SizeType smallOutsideSize;
  smallOutsideSize[0] = 10;
  smallOutsideSize[1] = 10;

  ImageType::IndexType smallOutsideStart;
  smallOutsideStart[0] = 110;
  smallOutsideStart[1] = 110;

  smallOutsideRegion.SetSize(smallOutsideSize);
  smallOutsideRegion.SetIndex(smallOutsideStart);

  std::cout << "Small outside region is " << bigRegion.IsInside(smallOutsideRegion) << std::endl;
  
  // Small overlap region
  ImageType::RegionType smallOverlapRegion;

  ImageType::SizeType smallOverlapSize;
  smallOverlapSize[0] = 10;
  smallOverlapSize[1] = 10;

  ImageType::IndexType smallOverlapStart;
  smallOverlapStart[0] = 97;
  smallOverlapStart[1] = 97;

  smallOverlapRegion.SetSize(smallOverlapSize);
  smallOverlapRegion.SetIndex(smallOverlapStart);

  std::cout << "Small overlap region is " << bigRegion.IsInside(smallOverlapRegion) << std::endl;
  
  return EXIT_SUCCESS;
}
