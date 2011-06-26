#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkTileImageFilter.h"

int main(int argc, char *argv[] )
{
  // Verify arguments
  if (argc < 4)
    {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << "input1 input2 output" << std::endl;
    return EXIT_FAILURE;
    }

  // Parse arguments
  std::string input1FileName = argv[1];
  std::string input2FileName = argv[2];
  std::string outputFileName = argv[3];

  // Output arguments
  std::cout << "input1FileName " << input1FileName << std::endl;
  std::cout << "input2FileName " << input2FileName << std::endl;;
  std::cout << "outputFileName " << outputFileName << std::endl;;
  
  typedef itk::Image< unsigned char, 2>   ImageType;

  // Read images
  typedef itk::ImageFileReader< ImageType > ImageReaderType ;
  ImageReaderType::Pointer reader1 = ImageReaderType::New();
  reader1->SetFileName(input1FileName);
  reader1->Update();

  ImageReaderType::Pointer reader2 = ImageReaderType::New();
  reader2->SetFileName(input2FileName);
  reader2->Update();

  // Tile the images side-by-side
  typedef itk::TileImageFilter< ImageType, ImageType > TileFilterType;

  TileFilterType::Pointer tileFilter = TileFilterType::New();

  itk::FixedArray< unsigned int, 2 > layout;

  layout[0] = 2;
  layout[1] = 0;

  tileFilter->SetLayout( layout );

  tileFilter->SetInput(0, reader1->GetOutput());
  tileFilter->SetInput(1, reader2->GetOutput());

  // Set the value of output pixels which are created by mismatched size inputu images.
  // If the two images are the same height, this will not be used.
  unsigned char fillerValue = 128;
  tileFilter->SetDefaultPixelValue( fillerValue );

  tileFilter->Update();

  // Write the output image
  typedef itk::ImageFileWriter< ImageType > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput( tileFilter->GetOutput() );
  writer->SetFileName( outputFileName );
  writer->Update();

  return EXIT_SUCCESS;
}
