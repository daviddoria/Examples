#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

#include "itkVnlFFTRealToComplexConjugateImageFilter.h"
#include "itkVnlFFTComplexConjugateToRealImageFilter.h"
#include "itkComplexToRealImageFilter.h"
#include "itkComplexToImaginaryImageFilter.h"
#include "itkRealAndImaginaryToComplexImageFilter.h"
#include "itkMultiplyByConstantImageFilter.h"
#include "itkMultiplyImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkFFTShiftImageFilter.h"
#include "itkMinimumMaximumImageCalculator.h"

int main(int argc, char*argv[])
{
  const    unsigned int    Dimension = 2;
  typedef  float           PixelType;
  typedef itk::Image< PixelType, Dimension >  FloatImageType;
  typedef itk::Image< unsigned char, Dimension >  UnsignedCharImageType;

  if( argc < 3 )
    {
    std::cerr << "Missing Parameters " << std::endl;
    std::cerr << "Usage: " << argv[0];
    std::cerr << " FixedImage MovingImage"<< std::endl;;
    return EXIT_FAILURE;
    }
  std::string fixedImageFilename = argv[1];
  std::string movingImageFilename = argv[2];

  // Read the input images
  typedef itk::ImageFileReader< FloatImageType  > ImageReaderType;
  ImageReaderType::Pointer  fixedImageReader  = ImageReaderType::New();
  fixedImageReader->SetFileName( fixedImageFilename );
  fixedImageReader->Update();

  ImageReaderType::Pointer movingImageReader = ImageReaderType::New();
  movingImageReader->SetFileName( movingImageFilename );
  movingImageReader->Update();

  // Shift the input images
  typedef itk::FFTShiftImageFilter< FloatImageType, FloatImageType > FFTShiftFilterType;
  FFTShiftFilterType::Pointer fixedFFTShiftFilter = FFTShiftFilterType::New();
  fixedFFTShiftFilter->SetInput(fixedImageReader->GetOutput());
  fixedFFTShiftFilter->Update();

  FFTShiftFilterType::Pointer movingFFTShiftFilter = FFTShiftFilterType::New();
  movingFFTShiftFilter->SetInput(movingImageReader->GetOutput());
  movingFFTShiftFilter->Update();

  // Compute the FFT of the input
  typedef itk::VnlFFTRealToComplexConjugateImageFilter<
                                PixelType, Dimension >  FFTFilterType;
  FFTFilterType::Pointer fixedFFTFilter = FFTFilterType::New();
  fixedFFTFilter->SetInput( fixedFFTShiftFilter->GetOutput() );
  fixedFFTFilter->Update();

  FFTFilterType::Pointer movingFFTFilter = FFTFilterType::New();
  movingFFTFilter->SetInput( movingFFTShiftFilter->GetOutput() );

  typedef FFTFilterType::OutputImageType    SpectralImageType;

  // Take the conjugate of the fftFilterMoving
  // Extract the real part
  typedef itk::ComplexToRealImageFilter<SpectralImageType, FloatImageType> RealFilterType;
  RealFilterType::Pointer realFilter = RealFilterType::New();
  realFilter->SetInput(movingFFTFilter->GetOutput());

  // Extract the imaginary part
  typedef itk::ComplexToImaginaryImageFilter<SpectralImageType, FloatImageType> ImaginaryFilterType;
  ImaginaryFilterType::Pointer imaginaryFilter = ImaginaryFilterType::New();
  imaginaryFilter->SetInput(movingFFTFilter->GetOutput());

  // Flip the sign of the imaginary and combine with the real part again
  typedef itk::MultiplyByConstantImageFilter<FloatImageType,PixelType,FloatImageType> MultiplyConstantFilterType;
  MultiplyConstantFilterType::Pointer flipSignFilter = MultiplyConstantFilterType::New();
  flipSignFilter->SetConstant(-1);
  flipSignFilter->SetInput(imaginaryFilter->GetOutput());
  typedef itk::RealAndImaginaryToComplexImageFilter<PixelType,PixelType,PixelType,2> RealImagToComplexFilterType;
  RealImagToComplexFilterType::Pointer conjugateFilter = RealImagToComplexFilterType::New();
  conjugateFilter->SetInput1(realFilter->GetOutput());
  conjugateFilter->SetInput2(flipSignFilter->GetOutput());

  // The conjugate product of the spectrum
  typedef itk::MultiplyImageFilter< SpectralImageType,
                                  SpectralImageType,
                                  SpectralImageType >  MultiplyFilterType;
  MultiplyFilterType::Pointer multiplyFilter = MultiplyFilterType::New();
  multiplyFilter->SetInput1( fixedFFTFilter->GetOutput() );
  multiplyFilter->SetInput2( conjugateFilter->GetOutput() );

  // IFFT
  typedef itk::VnlFFTComplexConjugateToRealImageFilter<
        PixelType, Dimension >  IFFTFilterType;
  IFFTFilterType::Pointer fftInverseFilter = IFFTFilterType::New();
  fftInverseFilter->SetInput( multiplyFilter->GetOutput() );

  // Write the spectrum
  typedef itk::RescaleIntensityImageFilter< FloatImageType,  UnsignedCharImageType > RescaleFilterType;
  RescaleFilterType::Pointer  rescaler =  RescaleFilterType::New();
  rescaler->SetInput( fftInverseFilter->GetOutput() );
  rescaler->SetOutputMinimum(0);
  rescaler->SetOutputMaximum(255);
  rescaler->Update();

  typedef itk::ImageFileWriter< UnsignedCharImageType >  WriterType;
  WriterType::Pointer writer =  WriterType::New();
  writer->SetFileName( "CrossCorr.png" );
  writer->SetInput( rescaler->GetOutput() );
  writer->Update();

  typedef itk::MinimumMaximumImageCalculator <UnsignedCharImageType>
          ImageCalculatorFilterType;

  ImageCalculatorFilterType::Pointer imageCalculatorFilter
          = ImageCalculatorFilterType::New ();
  imageCalculatorFilter->SetImage(rescaler->GetOutput());
  imageCalculatorFilter->Compute();

  UnsignedCharImageType::IndexType maximumLocation = imageCalculatorFilter->GetIndexOfMaximum();
  std::cout << maximumLocation << std::endl; // should be (17,15)

  /*
  if ypeak < size(I,1)/2 ypeak = -(ypeak-1);
  else ypeak = size(I,1) - (ypeak-1);
  end
  if xpeak < size(I,2)/2 xpeak = -(xpeak-1);
  else xpeak = size(I,2) - (xpeak-1);
  end
  */

  return EXIT_SUCCESS;
}