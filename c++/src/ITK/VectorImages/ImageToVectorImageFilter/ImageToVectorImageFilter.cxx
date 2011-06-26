#include "itkImage.h"
#include "itkImageToVectorImageFilter.h"
#include "itkVectorImage.h"

typedef itk::VectorImage<unsigned char, 2>  VectorImageType;
typedef itk::Image<unsigned char, 2>  ScalarImageType;

void CreateImage(ScalarImageType::Pointer image);

int main(int argc, char *argv[])
{
  ScalarImageType::Pointer image0 = ScalarImageType::New();
  CreateImage(image0);
  
  ScalarImageType::Pointer image1 = ScalarImageType::New();
  CreateImage(image1);
  
  ScalarImageType::Pointer image2 = ScalarImageType::New();
  CreateImage(image2);
  
  typedef itk::ImageToVectorImageFilter<ScalarImageType> ImageToVectorImageFilterType;
  ImageToVectorImageFilterType::Pointer imageToVectorImageFilter = ImageToVectorImageFilterType::New();
  imageToVectorImageFilter->SetNthInput(0, image0);
  imageToVectorImageFilter->SetNthInput(1, image1);
  imageToVectorImageFilter->SetNthInput(2, image2);
  imageToVectorImageFilter->Update();
  
  VectorImageType::Pointer vectorImage = imageToVectorImageFilter->GetOutput();
  
  return EXIT_SUCCESS;
}

void CreateImage(ScalarImageType::Pointer image)
{
  ScalarImageType::IndexType start;
  start.Fill(0);
 
  ScalarImageType::SizeType size;
  size.Fill(100);

  ScalarImageType::RegionType region(start,size); 
 
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);
  
}