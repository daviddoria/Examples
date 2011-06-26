#include <itkVectorContainer.h>
#include <itkPoint.h>

int main(int, char*[])
{
  typedef itk::Point<double, 2> PointType;
  typedef itk::VectorContainer<int, PointType> VectorContainerType;

  PointType p0;
  p0[0] = 1.0;
  p0[1] = 2.0;

  PointType p1;
  p1[0] = 2.0;
  p1[1] = 3.0;

  VectorContainerType::Pointer points = VectorContainerType::New();
  points->Reserve(2);
  VectorContainerType::Iterator point = points->Begin();
  point->Value() = p0;
  point++;
  point->Value() = p1;

  point = points->Begin();
  while(point != points->End())
    {
    std::cout << point->Value() << std::endl;
    point++;
    }

  return EXIT_SUCCESS;
}
