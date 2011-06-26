// NOTE: this doesn't work, see VectorIndexSelectionCastImageFilter instead

#include "itkVectorImage.h"
#include "itkImageRegionIterator.h"
#include "itkNthElementImageAdaptor.h"
#include "itkMinimumMaximumImageCalculator.h"

typedef itk::VectorImage<float, 2> VectorImageType;

void CreateImage(VectorImageType::Pointer image);

int main(int, char *[])
{
  VectorImageType::Pointer image = VectorImageType::New();
  CreateImage(image);

  typedef itk::NthElementImageAdaptor<VectorImageType, float> ImageAdaptorType;
  ImageAdaptorType::Pointer adaptor = ImageAdaptorType::New();
  adaptor->SelectNthElement(0);
  adaptor->SetImage(image);

  typedef itk::MinimumMaximumImageCalculator <ImageAdaptorType> ImageCalculatorFilterType;
  ImageCalculatorFilterType::Pointer imageCalculatorFilter = ImageCalculatorFilterType::New();
  imageCalculatorFilter->SetImage(adaptor);
  imageCalculatorFilter->Compute();

  return EXIT_SUCCESS;
}

void CreateImage(VectorImageType::Pointer image)
{
  VectorImageType::IndexType start;
  start.Fill(0);

  VectorImageType::SizeType size;
  size.Fill(2);

  VectorImageType::RegionType region(start, size);

  image->SetRegions(region);
  image->SetNumberOfComponentsPerPixel(3);
  image->Allocate();

  typedef itk::VariableLengthVector<double> VariableVectorType;
  VariableVectorType variableLengthVector;
  variableLengthVector.SetSize(3);
  variableLengthVector[0] = 1.1;
  variableLengthVector[1] = 2.2;
  variableLengthVector[2] = 3.3;

  image->FillBuffer(variableLengthVector);

}