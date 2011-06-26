#include "itkImage.h"
#include "itkImageDuplicator.h"
#include "itkRandomImageSource.h"

int main(int, char *[])
{
  typedef itk::Image< unsigned char, 2 >  ImageType;

  itk::RandomImageSource<ImageType>::Pointer randomImageSource =
    itk::RandomImageSource<ImageType>::New();
  randomImageSource->SetNumberOfThreads(1); // to produce non-random results

  randomImageSource->Update();
  ImageType::Pointer image = randomImageSource->GetOutput();

  typedef itk::ImageDuplicator< ImageType > DuplicatorType;
  DuplicatorType::Pointer duplicator = DuplicatorType::New();
  duplicator->SetInputImage(image);
  duplicator->Update();
  ImageType::Pointer clonedImage = duplicator->GetOutput();

  return EXIT_SUCCESS;
}