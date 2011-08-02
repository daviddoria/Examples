#include "itkImage.h"
#include "itkImageFileReader.h"

int main(int argc, char *argv[])
{
  if(argc < 2)
    {
    std::cerr << "Required: filename" << std::endl;

    return EXIT_FAILURE;
    }
  std::string inputFilename = argv[1];

  typedef itk::Image< unsigned char, 2 >  ImageType;

  typedef itk::ImageFileReader<ImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(inputFilename.c_str());
  reader->Update();
  
  std::cout << "Size: " << reader->GetOutput()->GetLargestPossibleRegion().GetSize() << std::endl;
  std::cout << "Total pixels: " << reader->GetOutput()->GetLargestPossibleRegion().GetSize()[0] * 
                                   reader->GetOutput()->GetLargestPossibleRegion().GetSize()[1] << std::endl;

  return EXIT_SUCCESS;
}
