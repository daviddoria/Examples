#include "itkVector.h"
#include "itkListSample.h"
#include "itkGaussianMixtureModelComponent.h"
#include "itkCovariantVector.h"
#include "itkImageToListSampleFilter.h"
#include "itkImageRegionIterator.h"

typedef itk::CovariantVector<unsigned char, 3> PixelType;

typedef itk::Image<PixelType, 2>  ImageType;

void CreateImage(ImageType::Pointer image);

int main(int argc, char*argv[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);
  
  typedef itk::Statistics::ImageToListSampleFilter<ImageType> ImageToListSampleFilterType;
  ImageToListSampleFilterType::Pointer imageToListSampleFilter = ImageToListSampleFilterType::New();
  imageToListSampleFilter->SetInput(image);
  imageToListSampleFilter->Update();
  
  itk::Array< double > params(12);
  
  for(unsigned int i = 0; i < 3; i++)
    {
    params[i] = i; // mean of dimension i
    }
    
  for(unsigned int i = 3; i < 12; i++)
    {
    params[i] = 5.0; // covariance
    //params[i] = i; // covariance
    }
    
    /*
  unsigned int counter = 0;
  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      if(i == j)
        {
        params[3+counter] = 5; // diagonal
        }
      else
        {
          params[3+counter] = 0; // off-diagonal
        }
      counter++;
      }
    }
    */

  std::cout << "Params: " << params << std::endl;
  
 typedef itk::Statistics::GaussianMixtureModelComponent< ImageToListSampleFilterType::ListSampleType >
    ComponentType;

  // Create the components
  ComponentType::Pointer component = ComponentType::New();
  component->SetSample( imageToListSampleFilter->GetOutput() );
  component->SetParameters(params);

  ImageToListSampleFilterType::ListSampleType::MeasurementVectorType mv(3);
  mv[0] = 0;
  mv[1] = 0;
  mv[2] = 0;
  double a = component->Evaluate(mv);
  //std::cout << component->Evaluate(mv) << std::endl;
  
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

  // Make a red and a green square
  itk::CovariantVector<unsigned char, 3> green;
  green[0] = 0;
  green[1] = 255;
  green[2] = 0;

  itk::CovariantVector<unsigned char, 3> red;
  red[0] = 255;
  red[1] = 0;
  red[2] = 0;

  itk::CovariantVector<unsigned char, 3> black;
  black[0] = 0;
  black[1] = 0;
  black[2] = 0;

  itk::ImageRegionIterator<ImageType> imageIterator(image,region);
  imageIterator.GoToBegin();

  while(!imageIterator.IsAtEnd())
    {
    if(imageIterator.GetIndex()[0] > 2 && imageIterator.GetIndex()[0] < 5 &&
      imageIterator.GetIndex()[1] > 2 && imageIterator.GetIndex()[1] < 5)
      {
      imageIterator.Set(green);
      }
    else if(imageIterator.GetIndex()[0] > 6 && imageIterator.GetIndex()[0] < 9 &&
      imageIterator.GetIndex()[1] > 6 && imageIterator.GetIndex()[1] < 9)
      {
      imageIterator.Set(red);
      }
    else
      {
      imageIterator.Set(black);
      }
    ++imageIterator;
    }

}
