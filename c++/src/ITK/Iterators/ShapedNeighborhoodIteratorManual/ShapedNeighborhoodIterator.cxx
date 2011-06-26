#include "itkImage.h"
#include "itkShapedNeighborhoodIterator.h"
#include "itkImageRegionIterator.h"
#include "itkNeighborhoodAlgorithm.h"
 
// Notice that char type pixel values will not appear 
// properly on the command prompt therefore for the 
// demonstration purposes it is best to use the int 
// type, however in real applications iterators have 
// no problems with char type images.
//typedef itk::Image<unsigned char, 2>  ImageType;
typedef itk::Image<unsigned int, 2>  ImageType;
 
void CreateImage(ImageType::Pointer image);
 
int main(int, char*[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);
  ImageType::SizeType radius;
  radius.Fill(1);
 
  typedef itk::ShapedNeighborhoodIterator<ImageType> IteratorType;
 
  IteratorType::OffsetType top = {{0,-1}};
  IteratorType::OffsetType bottom = {{0,1}};
  IteratorType::OffsetType left = {{-1,0}};
  IteratorType::OffsetType right = {{1,0}};
  IteratorType::OffsetType center = {{0,0}};
 
  typedef itk::NeighborhoodAlgorithm
    ::ImageBoundaryFacesCalculator< ImageType > FaceCalculatorType;
 
  FaceCalculatorType faceCalculator;
  FaceCalculatorType::FaceListType faceList;
  faceList = faceCalculator(image, image->GetLargestPossibleRegion(),
                            radius);
  FaceCalculatorType::FaceListType::iterator faceListIterator;
 
  for ( faceListIterator=faceList.begin(); faceListIterator != faceList.end(); ++faceListIterator)
    {
    IteratorType iterator(radius, image, *faceListIterator);
    iterator.ActivateOffset(top);
    iterator.ActivateOffset(bottom);
    iterator.ActivateOffset(left);
    iterator.ActivateOffset(right);
    iterator.ActivateOffset(center);
 
    for(iterator.GoToBegin(); !iterator.IsAtEnd(); ++iterator) // Crashes here!
      {
      // The method for accessing pixel values using neighborhood
      // iterators is GetPixel(offset).
      std::cout << "top: " << iterator.GetPixel(top) << std::endl;
      std::cout << "bottom: " << iterator.GetPixel(bottom) << std::endl;
      std::cout << "left: " << iterator.GetPixel(left) << std::endl;
      std::cout << "right: " << iterator.GetPixel(right) << std::endl;
      
      std::cout << "top: " << iterator[top] << std::endl;
      std::cout << "bottom: " << iterator[bottom] << std::endl;
      std::cout << "left: " << iterator[left] << std::endl;
      std::cout << "right: " << iterator[right] << std::endl;
      
      }
    }
 
  return EXIT_SUCCESS;
}
 
void CreateImage(ImageType::Pointer image)
{
  ImageType::IndexType start;
  start.Fill(0);
 
  ImageType::SizeType size;
  size.Fill(10);
  
  ImageType::RegionType region(start,size);

  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);
 
}