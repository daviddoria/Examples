#include "itkImage.h"
#include "itkTranslationTransform.h"
#include "itkImageFileReader.h"
#include "itkVectorResampleImageFilter.h"
#include "itkCovariantVector.h"
#include "itkNumericTraits.h"

int main(int argc, char *argv[])
{
  typedef itk::CovariantVector<double, 3> VectorType;
  typedef itk::Image<VectorType, 2>  VectorImageType;

  VectorImageType::Pointer image = VectorImageType::New();
  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(10);

  itk::ImageRegion<2> region(start,size);
  image->SetRegions(region);
  image->Allocate();
  //image->FillBuffer(itk::NumericTraits<VectorType>::Zero);

  itk::TranslationTransform<double,2>::Pointer transform =
    itk::TranslationTransform<double,2>::New();
  itk::TranslationTransform<double,2>::OutputVectorType translation;
  translation[0] = 10;
  translation[1] = 20;
  transform->Translate(translation);

  typedef itk::VectorResampleImageFilter< VectorImageType, VectorImageType > VectorResampleFilterType;
  VectorResampleFilterType::Pointer vectorResampleFilter = VectorResampleFilterType::New();
  vectorResampleFilter->SetInput(image);
  vectorResampleFilter->SetSize(image->GetLargestPossibleRegion().GetSize());
  vectorResampleFilter->SetTransform(transform);
  vectorResampleFilter->Update();
  
  std::cout << vectorResampleFilter->GetOutput()->GetLargestPossibleRegion() << std::endl;

  return EXIT_SUCCESS;
}
