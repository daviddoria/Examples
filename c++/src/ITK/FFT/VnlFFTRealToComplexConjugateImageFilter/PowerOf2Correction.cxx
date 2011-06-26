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

  // Compute the smallest power of two that is bigger than the image
  unsigned int powerOfTwo = 0;
  for(unsigned int i = 0; i < 10; i++)
    {
    if(pow(2, i) > reader->GetOutput()->GetLargestPossibleRegion().GetSize()[0] &&
       pow(2, i) > reader->GetOutput()->GetLargestPossibleRegion().GetSize()[1])
      {
      powerOfTwo = i;
      break;
      }
    }

  // Create an image bigger than the input image and that has dimensions which are powers of two
  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(pow(2,powerOfTwo));

  itk::ImageRegion<2> region(start, size);

  FloatImageType::Pointer image = FloatImageType::New();
  image->SetRegions(region);
  image->Allocate();

  // The image dimensions must be powers of two
  typedef itk::PasteImageFilter <FloatImageType, FloatImageType >
    PasteImageFilterType;
  itk::Index<2> destinationIndex;
  destinationIndex.Fill(0);

  PasteImageFilterType::Pointer pasteFilter
    = PasteImageFilterType::New ();
  pasteFilter->SetSourceImage(reader->GetOutput());
  pasteFilter->SetDestinationImage(image);
  pasteFilter->SetSourceRegion(reader->GetOutput()->GetLargestPossibleRegion());
  pasteFilter->SetDestinationIndex(destinationIndex);
  pasteFilter->Update();

  image->Graft(pasteFilter->GetOutput());

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
}#include "itkImage.h"
#include "itkVnlFFTRealToComplexConjugateImageFilter.h"
#include "itkComplexToRealImageFilter.h"
#include "itkComplexToImaginaryImageFilter.h"

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkRescaleIntensityImageFilter.h"
//cars.jpg cars_fft_real.jpg cars_fft_imaginary.jpg
//kirby.jpg kirby_fft_real.jpg kirby_fft_imaginary.jpg

//see also Filtering/FFTImageFilter.cxx

int main( int argc, char * argv [] )
{
	if( argc < 4 )
	{
		std::cerr << "Usage: " << argv[0] << " inputScalarImage  outputRealPartOfComplexImage outputRealImaginaryPartOfComplexImage" << std::endl;
	}

	typedef float  PixelType;
	const unsigned int Dimension = 2;

	typedef itk::Image< PixelType, Dimension > ImageType;

	typedef itk::VnlFFTRealToComplexConjugateImageFilter<
			PixelType, Dimension >  FFTFilterType;

	FFTFilterType::Pointer fftFilter = FFTFilterType::New();

	// The input to this filter can be taken from a reader, for example.

	typedef itk::ImageFileReader< ImageType >  ReaderType;
	ReaderType::Pointer reader = ReaderType::New();
	reader->SetFileName( argv[1] );

	fftFilter->SetInput( reader->GetOutput() );

	// The execution of the filter can be triggered by invoking the \code{Update()}
// method.  Since this invocation can eventually throw and exception, the call
// must be placed inside a try/catch block.

	try
	{
		fftFilter->Update();
	}
	catch( itk::ExceptionObject & excp )
	{
		std::cerr << "Error: " << std::endl;
		std::cerr << excp << std::endl;
		return EXIT_FAILURE;
	}

	// In general the output of the FFT filter will be a complex image. We can
// proceed to save this image in a file for further analysis. This can be done
// by simply instantiating an \doxygen{ImageFileWriter} using the trait of the
// output image from the FFT filter. We construct one instance of the writer
// and pass the output of the FFT filter as the input of the writer.

	typedef FFTFilterType::OutputImageType    ComplexImageType;

	typedef itk::ImageFileWriter< ComplexImageType > ComplexWriterType;

	ComplexWriterType::Pointer complexWriter = ComplexWriterType::New();
	complexWriter->SetFileName("complexImage.mhd");

	complexWriter->SetInput( fftFilter->GetOutput() );

	// Finally we invoke the \code{Update()} method placing inside a try/catch
// block.

	try
	{
		complexWriter->Update();
	}
	catch( itk::ExceptionObject & excp )
	{
		std::cerr << "Error: " << std::endl;
		std::cerr << excp << std::endl;
		return EXIT_FAILURE;
	}

	// In addition to saving the complex image into a file, we could also extract
// its real and imaginary parts for further analysis. This can be done with the
// \doxygen{ComplexToRealImageFilter} and the
// \doxygen{ComplexToImaginaryImageFilter}.
	//
// We instantiate first the ImageFilter that will help us to extract the real
// part from the complex image.  The \code{ComplexToRealImageFilter} takes as
// first template parameter the type of the complex image and as second
// template parameter it takes the type of the output image pixel. We create
// one instance of this filter and connect as its input the output of the FFT
// filter.
	//
// \index{itk::ComplexToRealImageFilter}

	typedef itk::ComplexToRealImageFilter<
			ComplexImageType, ImageType > RealFilterType;

	RealFilterType::Pointer realFilter = RealFilterType::New();

	realFilter->SetInput( fftFilter->GetOutput() );

	typedef unsigned char                           WritePixelType;
	typedef itk::Image< WritePixelType, Dimension > WriteImageType;

	// Since the range of intensities in the Fourier domain can be quite
// concentrated, it result convenient to rescale the image in order to
// visualize it. For this purpose we instantiate here a
// \doxygen{RescaleIntensityImageFilter} that will rescale the intensities of
// the \code{real} image into a range suitable for writing in a file. We also
// set the minimum and maximum values of the output to the range of the pixel
// type used for writing.

	typedef itk::RescaleIntensityImageFilter<
			ImageType,
   WriteImageType > RescaleFilterType;

   RescaleFilterType::Pointer intensityRescaler = RescaleFilterType::New();

   intensityRescaler->SetInput( realFilter->GetOutput() );

   intensityRescaler->SetOutputMinimum(  0  );
   intensityRescaler->SetOutputMaximum( 255 );


   typedef itk::ImageFileWriter< WriteImageType > WriterType;

   WriterType::Pointer writer = WriterType::New();

   writer->SetFileName( argv[2] );

   //writer->SetInput( intensityRescaler->GetOutput() );
   writer->SetInput( realFilter->GetOutput() );

   try
   {
	   writer->Update();
   }
   catch( itk::ExceptionObject & excp )
   {
	   std::cerr << "Error writing the real image: " << std::endl;
	   std::cerr << excp << std::endl;
	   return EXIT_FAILURE;
   }

   // We can now instantiate the ImageFilter that will help us to extract the
// imaginary part from the complex image.  The filter that we use here is the
// \doxygen{ComplexToImaginaryImageFilter}. It takes as first template
// parameter the type of the complex image and as second template parameter it
// takes the type of the output image pixel. An instance of the filter is
// created, and its input is connected to the output of the FFT filter.

   typedef FFTFilterType::OutputImageType    ComplexImageType;

   typedef itk::ComplexToImaginaryImageFilter<
		   ComplexImageType, ImageType > ImaginaryFilterType;

   ImaginaryFilterType::Pointer imaginaryFilter = ImaginaryFilterType::New();

   imaginaryFilter->SetInput( fftFilter->GetOutput() );

   // The Imaginary image can then be rescaled and saved into a file, just as we
// did with the Real part.

   intensityRescaler->SetInput( imaginaryFilter->GetOutput() );
   writer->SetFileName( argv[3] );

   try
   {
	   writer->Update();
   }
   catch( itk::ExceptionObject & excp )
   {
	   std::cerr << "Error writing the imaginary image: " << std::endl;
	   std::cerr << excp << std::endl;
	   return EXIT_FAILURE;
   }

// For the sake of illustrating the use of a \doxygen{ImageFileReader} on
// Complex images, here we instantiate a reader that will load the Complex
// image that we just saved. Note that nothing special is required in this
// case. The instantiation is done just the same as for any other type of
// image. Which once again illustrates the power of Generic Programming.

   typedef itk::ImageFileReader< ComplexImageType > ComplexReaderType;

   ComplexReaderType::Pointer complexReader = ComplexReaderType::New();

   complexReader->SetFileName("complexImage.mhd");
   complexReader->Update();


  // A way of testing the pixel type of an image in file is to
  // invoke the ImageIO object from the reader and then call
  // \code{GetPixelTypeAsString()}
   complexReader->GetImageIO()->GetPixelTypeAsString(
			     complexReader->GetImageIO()->GetPixelType() );


   return EXIT_SUCCESS;
}