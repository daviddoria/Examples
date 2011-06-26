#include "itkImage.h"
#include "itkGridImageSource.h"

int main(int, char *[])
{
  typedef itk::Image< unsigned char, 2 >  ImageType;

  itk::GridImageSource<ImageType>::Pointer gridImageSource =
    itk::GridImageSource<ImageType>::New();


  return EXIT_SUCCESS;
}