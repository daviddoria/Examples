#include "itkImage.h"
#include "itkShapedNeighborhoodIterator.h"
#include "itkImageRegionIterator.h"

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

  // Create a small region (entirely valid)
  ImageType::RegionType region;
  ImageType::IndexType start;
  start[0] = 5;
  start[1] = 5;

  ImageType::SizeType size;
  size[0] = 3;
  size[1] = 4;

  region.SetSize(size);
  region.SetIndex(start);
  IteratorType iterator(radius, image, region);

  IteratorType::OffsetType top = {{0,-1}};
  IteratorType::OffsetType bottom = {{0,1}};
  IteratorType::OffsetType left = {{-1,0}};
  IteratorType::OffsetType right = {{1,0}};
  IteratorType::OffsetType center = {{0,0}};

  iterator.ActivateOffset(top);
  iterator.ActivateOffset(bottom);
  iterator.ActivateOffset(left);
  iterator.ActivateOffset(right);
  iterator.ActivateOffset(center);

  iterator.GoToBegin();
  iterator.IsAtEnd();

  for(iterator.GoToBegin(); !iterator.IsAtEnd(); ++iterator)
    {
      /*
    std::cout << "top: " << iterator[top] << std::endl;
    std::cout << "bottom: " << iterator[bottom] << std::endl;
    std::cout << "left: " << iterator[left] << std::endl;
    std::cout << "right: " << iterator[right] << std::endl;
  */
    //iterator.Print(std::cout);
    //std::cout << iterator.GetIndex() << std::endl;
    iterator.GetPixel(top);
    //std::cout << iterator.GetPixelIndex(top) << " " << (int)iterator.GetPixel(top) << std::endl;
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
  size[0] = 20;
  size[1] = 20;

  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(0);
    ++imageIterator;
    }
}