#include "itkImage.h"
//#include "itkShapedNeighborhoodIterator.h"
#include "itkImageRegionIterator.h"
#include "itkNeighborhoodAlgorithm.h"
#include "itkImageFileWriter.h"

typedef itk::Image<unsigned char, 2>  ImageType;

void CreateImage(ImageType::Pointer image);

int main(int, char*[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  ImageType::SizeType radius;
  radius.Fill(1);

  typedef itk::ImageRegionConstIterator<ImageType> IteratorType;

  typedef itk::NeighborhoodAlgorithm
    ::ImageBoundaryFacesCalculator< ImageType > FaceCalculatorType;

  FaceCalculatorType faceCalculator;
 
  FaceCalculatorType::FaceListType faceList;
  faceList = faceCalculator(image, image->GetLargestPossibleRegion(),
                            radius);
  
  FaceCalculatorType::FaceListType::iterator faceListIterator = faceList.begin();;
  // Do nothing with the center of the image
  ++faceListIterator;

  // Iterate over all of the boundary regions setting their value to white

  typedef itk::ImageRegionIterator<ImageType> OutputIteratorType;
  
  while(faceListIterator != faceList.end())
    {
    OutputIteratorType outputFaceIterator(image,*faceListIterator);
    outputFaceIterator.GoToBegin();
    while(!outputFaceIterator.IsAtEnd())
      {
      outputFaceIterator.Set(255);
      ++outputFaceIterator;
      }
    ++faceListIterator;
    }

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("test.png");
  writer->SetInput(image);
  writer->Update();

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(50);

  ImageType::RegionType region(start, size);

  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);
  
  itk::ImageRegionIterator<ImageType> imageIterator(image,region);

}