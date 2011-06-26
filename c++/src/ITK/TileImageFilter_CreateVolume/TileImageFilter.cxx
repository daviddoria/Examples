#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkTileImageFilter.h"

int main(int argc, char *argv[] )
{

  typedef unsigned char  PixelType;
  const unsigned int InputImageDimension = 2;
  const unsigned int OutputImageDimension = 3;

  typedef itk::Image< PixelType, InputImageDimension  >   InputImageType;
  typedef itk::Image< PixelType, OutputImageDimension >   OutputImageType;

  typedef itk::ImageFileReader< InputImageType > ImageReaderType ;

  typedef itk::TileImageFilter< InputImageType, OutputImageType > TilerType;

  typedef itk::ImageFileWriter< OutputImageType > WriterType;

  if (argc < 4)
    {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << "input1 input2 ... inputn output" << std::endl;
    return EXIT_FAILURE;
    }

  TilerType::Pointer tiler = TilerType::New();

  itk::FixedArray< unsigned int, OutputImageDimension > layout;

  layout[0] = 2;
  layout[1] = 2;
  layout[2] = 0;

  tiler->SetLayout( layout );

  unsigned int inputImageNumber = 0;

  ImageReaderType::Pointer reader = ImageReaderType::New();

  InputImageType::Pointer inputImageTile;

  for (int i = 1; i < argc - 1; i++)
    {
    reader->SetFileName( argv[i] );
    reader->UpdateLargestPossibleRegion();
    inputImageTile = reader->GetOutput();
    inputImageTile->DisconnectPipeline();
    tiler->SetInput( inputImageNumber++, inputImageTile );
    }

  PixelType filler = 128;

  tiler->SetDefaultPixelValue( filler );

  tiler->Update();

  WriterType::Pointer writer = WriterType::New();

  writer->SetInput( tiler->GetOutput() );

  writer->SetFileName( argv[argc-1] );

  try
    {
    writer->Update();
    }
  catch( itk::ExceptionObject & excp )
    {
    std::cerr << excp << std::endl;
    return EXIT_FAILURE;
    }

  return EXIT_SUCCESS;
}
