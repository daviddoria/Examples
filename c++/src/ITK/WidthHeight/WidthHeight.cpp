#include "itkImage.h"
#include "itkImageFileReader.h"

int main(int, char *[])
{
  // Create a vector image
  typedef itk::Image<float, 2>  ImageType;

  typedef itk::ImageFileReader<ImageType> ReaderType;

  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName("test.png");
  reader->Update();

  std::cout << reader->GetOutput()->GetLargestPossibleRegion().GetSize()[0] << " "
            << reader->GetOutput()->GetLargestPossibleRegion().GetSize()[1] << std::endl;

  // An example image had w = 200 and h = 100 (it is wider than it is tall). The above output
  // 200 100
  // so w = GetSize()[0]
  // and h = GetSize()[1]

  // A pixel inside the region
  itk::Index<2> indexInside;
  indexInside[0] = 150;
  indexInside[1] = 50;
  std::cout << reader->GetOutput()->GetLargestPossibleRegion().IsInside(indexInside) << std::endl;

  // A pixel outside the region
  itk::Index<2> indexOutside;
  indexOutside[0] = 50;
  indexOutside[1] = 150;
  std::cout << reader->GetOutput()->GetLargestPossibleRegion().IsInside(indexOutside) << std::endl;

  // This means that the [0] component of the index is referencing the left to right (x) value
  // and the [1] component of Index is referencing the top to bottom (y) value

  return EXIT_SUCCESS;
}
