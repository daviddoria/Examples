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

  typedef itk::ShapedNeighborhoodIterator<ImageType> IteratorType;

  IteratorType::OffsetType top = {{0,-1}};
  IteratorType::OffsetType bottom = {{0,1}};
  IteratorType::OffsetType left = {{-1,0}};
  IteratorType::OffsetType right = {{1,0}};
  //IteratorType::OffsetType center = {{0,0}};

  IteratorType iterator(radius, image, image->GetLargestPossibleRegion());
  iterator.ClearActiveList();
  iterator.ActivateOffset(top);
  iterator.ActivateOffset(bottom);
  iterator.ActivateOffset(left);
  iterator.ActivateOffset(right);
  //iterator.ActivateOffset(center);

  for(iterator.GoToBegin(); !iterator.IsAtEnd(); ++iterator)
    {
    //iterator.Print(std::cout);

    IteratorType::IndexListType::const_iterator indexIterator = iterator.GetActiveIndexList().begin();
    while (indexIterator != iterator.GetActiveIndexList().end())
      {
      //std::cout << *indexIterator << " ";
      //std::cout << (int)iterator.GetPixel(*indexIterator) << " ";
      std::cout << (int)iterator[*indexIterator][0] << " ";
      ++indexIterator;
      }
    std::cout << std::endl;


    //std::cout << "top: " << (int)iterator[top] << std::endl;
    //bool inbounds;
    //unsigned char pixel = iterator.GetPixel(top, inbounds);
    //std::cout << "top inbounds? " << inbounds << " " << (int)pixel << " " << static_cast<int>(iterator[top][0]) << std::endl;
    /*
    std::cout << "top: " << (int)iterator[top] << std::endl;
    std::cout << "bottom: " << (int)iterator[bottom] << std::endl;
    std::cout << "left: " << (int)iterator[left] << std::endl;
    std::cout << "right: " << (int)iterator[right] << std::endl;
    */
    }

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  // Create an image
  ImageType::RegionType region;
  ImageType::IndexType start;
  start[0] = 0;
  start[1] = 0;

  ImageType::SizeType size;
  size[0] = 10;
  size[1] = 10;

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