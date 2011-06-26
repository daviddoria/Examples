/*
 * I don't understand the usage of this - it only outputs an image, not a labelMap, so what is the point?
 */

#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIterator.h"
#include "itkBinaryShapeKeepNObjectsImageFilter.h"

typedef itk::Image<unsigned char, 2>  ImageType;
void CreateImage(ImageType::Pointer image);

int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  typedef itk::BinaryShapeKeepNObjectsImageFilter<ImageType> BinaryShapeKeepNObjectsImageFilterType;
  BinaryShapeKeepNObjectsImageFilterType::Pointer binaryShapeKeepNObjectsImageFilter = BinaryShapeKeepNObjectsImageFilterType::New();
  binaryShapeKeepNObjectsImageFilter->SetInput(image);
  binaryShapeKeepNObjectsImageFilter->SetNumberOfObjects(2);
  binaryShapeKeepNObjectsImageFilter->SetAttribute("PhysicalSize");
  binaryShapeKeepNObjectsImageFilter->SetAttribute(BinaryShapeKeepNObjectsImageFilterType::LabelObjectType::PHYSICAL_SIZE);
  binaryShapeKeepNObjectsImageFilter->Update();

  // Even though the input image has 3 objects, we should only see "2" here
  //std::cout << "There are " << binaryShapeKeepNObjectsImageFilter->GetOutput()->GetNumberOfLabelObjects() << " objects." << std::endl;
  for(unsigned int i = 0; i < binaryShapeKeepNObjectsImageFilter->GetNumberOfObjects(); i++)
    {
    //BinaryShapeKeepNObjectsImageFilterType::LabelObjectType* labelObject = binaryShapeKeepNObjectsImageFilter->GetOutput()->GetNthLabelObject(i);
    //binaryShapeKeepNObjectsImageFilter->GetOutput()->GetNthLabelObject(i);
    // Output the bounding box (an example of one possible property) of the ith region
    //std::cout << "Object " << i << " has bounding box " << labelObject->GetBoundingBox() << std::endl;
    }

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  // Create a black image with 3 white squares of different sizes
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(100);

  ImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);
  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,image->GetLargestPossibleRegion());
  
  // Make a square
  imageIterator.GoToBegin();
  while(!imageIterator.IsAtEnd())
    {
    if((imageIterator.GetIndex()[0] > 5 && imageIterator.GetIndex()[0] < 10) &&
      (imageIterator.GetIndex()[1] > 5 && imageIterator.GetIndex()[1] < 10) )
        {
        imageIterator.Set(255);
        }
      else
        {
        imageIterator.Set(0);
        }
    ++imageIterator;
    }
    
  // Make a square
  imageIterator.GoToBegin();
  while(!imageIterator.IsAtEnd())
    {
    if((imageIterator.GetIndex()[0] > 20 && imageIterator.GetIndex()[0] < 30) &&
      (imageIterator.GetIndex()[1] > 20 && imageIterator.GetIndex()[1] < 30) )
        {
        imageIterator.Set(255);
        }
      else
        {
        imageIterator.Set(0);
        }
    ++imageIterator;
    }
    
  // Make a square
  imageIterator.GoToBegin();
  while(!imageIterator.IsAtEnd())
    {
    if((imageIterator.GetIndex()[0] > 50 && imageIterator.GetIndex()[0] < 70) &&
      (imageIterator.GetIndex()[1] > 50 && imageIterator.GetIndex()[1] < 70) )
        {
        imageIterator.Set(255);
        }
      else
        {
        imageIterator.Set(0);
        }
    ++imageIterator;
    }

}