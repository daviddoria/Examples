#include "itkImage.h"
#include "itkCovariantVector.h"
#include "itkGradientImageFilter.h"
#include "itkImageRegionIterator.h"

int main(int argc, char * argv[])
{
  // Setup types
  typedef itk::CovariantVector<float, 2> VectorType;
  typedef itk::Image<VectorType, 2>   VectorImageType;
  VectorImageType::Pointer image = VectorImageType::New();

  itk::Size<2> size;
  size[0] = 5;
  size[1] = 5;

  itk::Index<2> index;
  index[0] = 0;
  index[1] = 0;

  VectorImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(index);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<VectorImageType> iterator(image, region);

  while(!iterator.IsAtEnd())
    {
    }
    
  // Create and setup a gradient filter
  typedef itk::GradientImageFilter<
    VectorImageType, float>  GradientFilterType;
      //VectorImageType, VectorType>  GradientFilterType;
  GradientFilterType::Pointer gradientFilter = GradientFilterType::New();
  gradientFilter->SetInput(image);
  gradientFilter->Update();

  return EXIT_SUCCESS;
}
