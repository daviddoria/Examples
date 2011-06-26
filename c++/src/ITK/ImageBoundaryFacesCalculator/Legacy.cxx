#include "itkImage.h"
#include "itkShapedNeighborhoodIterator.h"
#include "itkImageRegionIterator.h"
#include "itkNeighborhoodAlgorithm.h"

typedef itk::Image<unsigned char, 2>  ImageType;

void CreateImage(ImageType::Pointer image);

int main(int, char*[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  ImageType::SizeType radius;
  radius[0] = 1;
  radius[1] = 1;

  typedef itk::NeighborhoodIterator<ImageType> IteratorType;

  IteratorType::OffsetType top = {{0,-1}};
  IteratorType::OffsetType bottom = {{0,1}};
  IteratorType::OffsetType left = {{-1,0}};
  IteratorType::OffsetType right = {{1,0}};

  typedef itk::NeighborhoodAlgorithm
    ::ImageBoundaryFacesCalculator< ImageType > FaceCalculatorType;

  FaceCalculatorType faceCalculator;

  FaceCalculatorType::FaceListType faceList;
  faceList = faceCalculator(image, image->GetLargestPossibleRegion(),
                            radius);

  FaceCalculatorType::FaceListType::iterator faceListIterator;

  unsigned int regionCounter = 0;
  for ( faceListIterator=faceList.begin(); faceListIterator != faceList.end(); ++faceListIterator)
    {
    //std::cout << "Region " << regionCounter << std::endl;
    IteratorType iterator(radius, image,*faceListIterator);
    iterator.SetRadius(radius);
    unsigned int pixelCounter = 0;
    for(iterator.GoToBegin(); !iterator.IsAtEnd(); ++iterator)
      {
      bool topInBounds;
      bool bottomInBounds;
      bool leftInBounds;
      bool rightInBounds;
      //iterator.GetPixel(top, topInBounds);
      iterator.GetPixel(top);
      iterator.GetPixel(bottom, bottomInBounds);
      iterator.GetPixel(left, leftInBounds);
      iterator.GetPixel(right, rightInBounds);
      //std::cout << "top: " << topInBounds << " " << (int)iterator[top][0] << std::endl;
      //std::cout << "bottom: " << bottomInBounds << " " << (int)iterator[bottom][0] << std::endl;
      //std::cout << "left: " << leftInBounds << " " << (int)iterator[left][0] << std::endl;
      //std::cout << "right: " << rightInBounds << " " << (int)iterator[right][0] << std::endl;
      //std::cout << std::endl;
      pixelCounter++;
      }
    std::cout << "Region " << regionCounter << " had " << pixelCounter << " pixels." << std::endl;
    regionCounter++;
    }

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  // Create an image with 2 connected components
  ImageType::RegionType region;
  ImageType::IndexType start;
  start[0] = 0;
  start[1] = 0;

  ImageType::SizeType size;
  size[0] = 5;
  size[1] = 5;

  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(2);
    ++imageIterator;
    }

}