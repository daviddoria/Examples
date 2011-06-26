#include "itkVectorImage.h"
#include "itkImageFileReader.h"

int main(int argc, char *argv[])
{
  if(argc < 2)
    {
    std::cerr << "Required: filename" << std::endl;

    return EXIT_FAILURE;
    }
  std::string inputFilename = argv[1];

  typedef itk::ImageIOBase::IOComponentType ScalarPixelType;

  itk::ImageIOBase::Pointer imageIO =
        itk::ImageIOFactory::CreateImageIO(
            inputFilename.c_str(), itk::ImageIOFactory::ReadMode);
  imageIO->SetFileName(inputFilename);
  imageIO->ReadImageInformation();

  const ScalarPixelType pixelType = imageIO->GetComponentType();
  const int numberOfComponents = imageIO->GetNumberOfComponents();

  typedef itk::VectorImage<pixelType, 2> ImageType;

  ImageType::Pointer image = ImageType::New();
  image->SetNumberOfComponentsPerPixel(numberOfComponents);

  typedef itk::ImageFileReader<TImageType> ReaderType;
  typename ReaderType::Pointer reader = ReaderType::New();

  reader->SetFileName(inputFilename);
  reader->Update();

  image->Graft(reader->GetOutput());


  return EXIT_SUCCESS;
}
