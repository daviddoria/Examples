#include "itkImageFileWriter.h"
#include "itkResampleVectorImageFilter.h"
#include "itkVectorImage.h"
#include "itkTranslationTransform.h"

typedef itk::VectorImage<double, 2> VectorImageType;

static void CreateImage(VectorImageType::Pointer image);

int main(int argc, char *argv[])
{
  VectorImageType::Pointer image = VectorImageType::New();
  CreateImage(image);
  
  //std::cout << image->GetLargestPossibleRegion() << std::endl;
  
  typedef itk::TranslationTransform<double,2> TranslationTransformType;
  TranslationTransformType::Pointer transform =
    TranslationTransformType::New();
  TranslationTransformType::OutputVectorType translation;
  translation[0] = 10;
  translation[1] = 20;
  transform->Translate(translation);

  typedef itk::ResampleVectorImageFilter< VectorImageType, VectorImageType > ResampleVectorImageFilterType;
  ResampleVectorImageFilterType::Pointer resampleVectorImageFilter = ResampleVectorImageFilterType::New();
  resampleVectorImageFilter->SetInput(image);
  resampleVectorImageFilter->SetSize(image->GetLargestPossibleRegion().GetSize());
  /*
  itk::Index<2> outputStartIndex;
  outputStartIndex[0] = 10;
  outputStartIndex[1] = 20;
  resampleVectorImageFilter->SetOutputStartIndex(outputStartIndex);
  itk::Point<float, 2> outputOrigin;
  outputOrigin[0] = 10;
  outputOrigin[1] = 20;
  resampleVectorImageFilter->SetOutputOrigin(outputOrigin);
  */
  resampleVectorImageFilter->SetTransform(transform.GetPointer());
  resampleVectorImageFilter->Update();

  //std::cout << resampleVectorImageFilter->GetOutput()->GetLargestPossibleRegion() << std::endl;
  
  typedef  itk::ImageFileWriter< VectorImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("output.mhd");
  writer->SetInput(resampleVectorImageFilter->GetOutput());
  writer->Update();

  return EXIT_SUCCESS;
}

void CreateImage(VectorImageType::Pointer image)
{
  itk::Index<2> start;
  start.Fill(0);
 
  itk::Size<2> size;
  size.Fill(100);
 
  itk::ImageRegion<2> region(start,size);
 
  image->SetRegions(region);
  image->SetNumberOfComponentsPerPixel(3);
  image->Allocate();
  //image->FillBuffer(itk::NumericTraits<VectorImageType::PixelType>::Zero);
 
  itk::ImageRegionIterator<VectorImageType> imageIterator(image,region);
 
  VectorImageType::PixelType zeroPixel;
  zeroPixel.SetSize(3);
  zeroPixel.Fill(0);
  
  VectorImageType::PixelType pixel;
  pixel.SetSize(3);
  pixel.Fill(100);
  
  while(!imageIterator.IsAtEnd())
    {
    if(imageIterator.GetIndex()[0] < 70)
      {
      imageIterator.Set(pixel);
      }
    else
      {
      imageIterator.Set(zeroPixel);
      }
 
    ++imageIterator;
    }

  typedef  itk::ImageFileWriter< VectorImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("input.mhd");
  writer->SetInput(image);
  writer->Update();

}
