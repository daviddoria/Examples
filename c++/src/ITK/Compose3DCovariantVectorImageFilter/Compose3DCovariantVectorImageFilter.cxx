#include "itkImageAdaptor.h"
#include "itkImageRegionIterator.h"
#include "itkCompose3DCovariantVectorImageFilter.h"

typedef itk::Image<itk::CovariantVector< float, 3>, 2> VectorImageType;
typedef itk::Image<float, 2> ScalarImageType;

void CreateImage(ScalarImageType::Pointer image);

int main(int, char *[])
{
  ScalarImageType::Pointer image = ScalarImageType::New();
  CreateImage(image);

  typedef itk::Compose3DCovariantVectorImageFilter<ScalarImageType,
                              VectorImageType> ComposeCovariantVectorImageFilterType;

  ComposeCovariantVectorImageFilterType::Pointer composeFilter = ComposeCovariantVectorImageFilterType::New();

  composeFilter->SetInput1(image);
  composeFilter->SetInput2(image);
  composeFilter->SetInput3(image);
  composeFilter->Update();

  itk::Index<2> index;
  index.Fill(0);

  std::cout << image->GetPixel(index) << std::endl;

  std::cout << composeFilter->GetOutput()->GetPixel(index) << std::endl;

  return EXIT_SUCCESS;
}

void CreateImage(ScalarImageType::Pointer image)
{
  ScalarImageType::IndexType start;
  start.Fill(0);

  ScalarImageType::SizeType size;
  size.Fill(2);

  ScalarImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ScalarImageType> imageIterator(image,image->GetLargestPossibleRegion());

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(1.2);

    ++imageIterator;
    }
}