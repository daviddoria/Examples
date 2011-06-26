#include "itkImage.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkVnlFFTRealToComplexConjugateImageFilter.h"
#include "itkComplexToRealImageFilter.h"
#include "itkComplexToImaginaryImageFilter.h"
#include "itkComplexToModulusImageFilter.h"
#include "itkImageFileReader.h"
#include "itkCastImageFilter.h"
#include "itkPasteImageFilter.h"

#include <itksys/SystemTools.hxx>
#include "vnl/vnl_sample.h"
#include <math.h>

#include <itkImageToVTKImageFilter.h>

#include "QuickView.h"

int main(int argc, char*argv[])
{
  // Verify input
  if(argc < 2)
    {
    std::cerr << "Required: filename" << std::endl;
    return EXIT_FAILURE;
    }

  // Define some types
  typedef itk::Image<unsigned char, 2> UnsignedCharImageType;
  typedef itk::Image<float, 2> FloatImageType;

  // Read the image
  typedef itk::ImageFileReader<FloatImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(argv[1]);
  reader->Update();

  FloatImageType::Pointer image = FloatImageType::New();
  image->Graft(reader->GetOutput());

  // Compute the FFT
  typedef itk::VnlFFTRealToComplexConjugateImageFilter<FloatImageType::PixelType, 2> FFTType;
  FFTType::Pointer fftFilter = FFTType::New();
  fftFilter->SetInput(image);
  fftFilter->Update();

  // Extract the real part
  typedef itk::ComplexToRealImageFilter<FFTType::OutputImageType, FloatImageType> RealFilterType;
  RealFilterType::Pointer realFilter = RealFilterType::New();
  realFilter->SetInput(fftFilter->GetOutput());
  realFilter->Update();

  typedef itk::RescaleIntensityImageFilter< FloatImageType, UnsignedCharImageType > RescaleFilterType;
  RescaleFilterType::Pointer realRescaleFilter = RescaleFilterType::New();
  realRescaleFilter->SetInput(realFilter->GetOutput());
  realRescaleFilter->SetOutputMinimum(0);
  realRescaleFilter->SetOutputMaximum(255);
  realRescaleFilter->Update();

  // Extract the real part
  typedef itk::ComplexToImaginaryImageFilter<FFTType::OutputImageType, FloatImageType> ImaginaryFilterType;
  ImaginaryFilterType::Pointer imaginaryFilter = ImaginaryFilterType::New();
  imaginaryFilter->SetInput(fftFilter->GetOutput());
  imaginaryFilter->Update();

  RescaleFilterType::Pointer imaginaryRescaleFilter = RescaleFilterType::New();
  imaginaryRescaleFilter->SetInput(imaginaryFilter->GetOutput());
  imaginaryRescaleFilter->SetOutputMinimum(0);
  imaginaryRescaleFilter->SetOutputMaximum(255);
  imaginaryRescaleFilter->Update();

  // Compute the magnitude
  typedef itk::ComplexToModulusImageFilter<FFTType::OutputImageType, FloatImageType> ModulusFilterType;
  ModulusFilterType::Pointer modulusFilter = ModulusFilterType::New();
  modulusFilter->SetInput(fftFilter->GetOutput());
  modulusFilter->Update();

  RescaleFilterType::Pointer magnitudeRescaleFilter = RescaleFilterType::New();
  magnitudeRescaleFilter->SetInput(modulusFilter->GetOutput());
  magnitudeRescaleFilter->SetOutputMinimum(0);
  magnitudeRescaleFilter->SetOutputMaximum(255);
  magnitudeRescaleFilter->Update();

  QuickView viewer;
  viewer.AddImage(image.GetPointer());
  viewer.AddImage(realRescaleFilter->GetOutput());
  viewer.AddImage(imaginaryRescaleFilter->GetOutput());
  viewer.AddImage(magnitudeRescaleFilter->GetOutput());
  viewer.Visualize();

  return EXIT_SUCCESS;
}