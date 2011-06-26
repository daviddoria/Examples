#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkLaplacianRecursiveGaussianImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkTileImageFilter.h"

#include <iomanip> // setfill

int main(int argc, char * argv[])
{
  // Verify command line arguments
  if( argc < 2 )
    {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << "input" << std::endl;
    return EXIT_FAILURE;
    }

  // Parse command line arguments
  std::string inputFileName = argv[1];

  // Setup types
  typedef itk::Image< float,  2 >   FloatImageType;
  typedef itk::Image< unsigned char, 2 >   UnsignedCharImageType;

  typedef itk::ImageFileReader< UnsignedCharImageType >  readerType;

  typedef itk::LaplacianRecursiveGaussianImageFilter<
		  UnsignedCharImageType, FloatImageType >  filterType;

  // Create and setup a reader
  readerType::Pointer reader = readerType::New();
  reader->SetFileName( inputFileName.c_str() );

  typedef itk::Image< float, 3 >   OutputImageType;
  typedef itk::TileImageFilter< FloatImageType, OutputImageType > TilerType;

  
  TilerType::Pointer tileImageFilter = TilerType::New();

  itk::FixedArray< unsigned int, OutputImageType::ImageDimension > layout;

  layout[0] = 1;
  layout[1] = 1;
  layout[2] = 0;

  tileImageFilter->SetLayout( layout );
  
  // Setup the list of sigmas we want to use
  std::vector<float> sigmas;
  for(unsigned int i = 1; i < 50; ++i)
    {
    sigmas.push_back(i);
    }

  for(unsigned int sigmaId = 0; sigmaId < sigmas.size(); ++sigmaId)
    {
    std::cout << "Creating slice " << sigmaId << std::endl;
  
    // Create and setup a derivative filter
    filterType::Pointer LoGFilter = filterType::New();
    LoGFilter->SetInput( reader->GetOutput() );
    LoGFilter->SetSigma(sigmas[sigmaId]);
    LoGFilter->SetNormalizeAcrossScale(true);
    LoGFilter->Update();

    // The maximum will occur at blobs of diameter = Sigma

    tileImageFilter->SetInput( sigmaId, LoGFilter->GetOutput() );
    // To visualize the derivative, we must rescale the values
    // to a reasonable range
//     typedef itk::RescaleIntensityImageFilter<
// 		    FloatImageType, UnsignedCharImageType > RescaleFilterType;
// 
//     RescaleFilterType::Pointer rescaleFilter = RescaleFilterType::New();
//     rescaleFilter->SetOutputMinimum(0);
//     rescaleFilter->SetOutputMaximum(255);
//     rescaleFilter->SetInput( LoGFilter->GetOutput() );
//     rescaleFilter->Update();
// 
//     std::stringstream ss;
//     ss << std::setfill('0') << std::setw(2) << sigmas[sigmaId];
//     typedef  itk::ImageFileWriter< UnsignedCharImageType  > WriterType;
//     WriterType::Pointer writer = WriterType::New();
//     writer->SetFileName("sigma_" + ss.str() + ".png");
//     writer->SetInput(rescaleFilter->GetOutput());
//     writer->Update();
     }
  
  tileImageFilter->Update();
  
  typedef  itk::ImageFileWriter< OutputImageType  > OutputWriterType;
  OutputWriterType::Pointer outputWriter = OutputWriterType::New();
  outputWriter->SetFileName("full.mhd");
  outputWriter->SetInput(tileImageFilter->GetOutput());
  outputWriter->Update();
  
  return EXIT_SUCCESS;
}
