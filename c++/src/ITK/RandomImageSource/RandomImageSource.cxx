#include "itkImage.h"
#include "itkRandomImageSource.h"

#include "QuickView.h"

int main(int, char *[])
{
  typedef itk::Image< unsigned char, 2 >  ImageType;

  itk::Size<2> size;
  size.Fill(10);

  itk::RandomImageSource<ImageType>::Pointer randomImageSource =
    itk::RandomImageSource<ImageType>::New();
  randomImageSource->SetNumberOfThreads(1); // to produce non-random results
  randomImageSource->SetSize(size);
  randomImageSource->Update();

  QuickView viewer;
  viewer.AddImage(randomImageSource->GetOutput());
  viewer.Visualize();

  return EXIT_SUCCESS;
}