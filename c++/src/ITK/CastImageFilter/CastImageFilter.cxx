#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkCastImageFilter.h"

int main(int argc, char *argv[])
{
  if(argc < 2)
    {
    std::cerr << "Required: filename" << std::endl;
    return EXIT_FAILURE;
    }

  typedef itk::Image<unsigned char, 2>  UnsignedCharImageType;
  typedef itk::Image<float, 2>  FloatImageType;

  typedef itk::ImageFileReader<FloatImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(argv[1]);

  typedef itk::CastImageFilter< FloatImageType, UnsignedCharImageType > CastFilterType;
  CastFilterType::Pointer castFilter = CastFilterType::New();
  castFilter->SetInput(reader->GetOutput());
  castFilter->Update();

  return EXIT_SUCCESS;
}