#include "itkImageAdaptor.h"
#include "itkImageRegionIterator.h"
#include "itkNthElementImageAdaptor.h"
#include "itkBinomialBlurImageFilter.h"

typedef itk::Image<itk::CovariantVector< float, 3>, 2> VectorImageType;

void CreateImage(VectorImageType::Pointer image);

int main(int, char *[])
{
  VectorImageType::Pointer image = VectorImageType::New();
  CreateImage(image);

  typedef itk::NthElementImageAdaptor<VectorImageType,
                              float> ImageAdaptorType;

  ImageAdaptorType::Pointer adaptor = ImageAdaptorType::New();

  adaptor->SelectNthElement(0);
  adaptor->SetImage(image);

  typedef itk::BinomialBlurImageFilter<ImageAdaptorType, itk::Image<float,2> >  BlurFilterType;
  BlurFilterType::Pointer blurFilter = BlurFilterType::New();
  blurFilter->SetInput(adaptor);
  blurFilter->Update();

  return EXIT_SUCCESS;
}

void CreateImage(VectorImageType::Pointer image)
{
  VectorImageType::IndexType start;
  start.Fill(0);

  VectorImageType::SizeType size;
  size.Fill(2);

  VectorImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<VectorImageType> imageIterator(image,image->GetLargestPossibleRegion());
  itk::CovariantVector<float, 3> vec;
  vec[0] = 1;
  vec[1] = 2;
  vec[2] = 3;

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(vec);

    ++imageIterator;
    }
}