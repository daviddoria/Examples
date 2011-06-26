#include <itkBoundingBox.h>
#include <itkPoint.h>
#include <itkVectorContainer.h>

#include <vector>

int main(int, char*[])
{
  typedef itk::Point<double, 2> PointType;
  PointType p0;
  p0[0] = 0.0;
  p0[1] = 0.0;

  PointType p1;
  p1[0] = 2.0;
  p1[1] = 0.0;

  PointType p2;
  p2[0] = 0.0;
  p2[1] = 1.0;

  typedef itk::VectorContainer<int, PointType> VectorContainerType;
  VectorContainerType::Pointer points = VectorContainerType::New();
  points->Reserve(3);
  VectorContainerType::Iterator point = points->Begin();
  point->Value() = p0;
  point++;
  point->Value() = p1;
  point++;
  point->Value() = p2;

  typedef  itk::BoundingBox<int, 2, double, VectorContainerType> BoundingBoxType;
  BoundingBoxType::Pointer boundingBox = BoundingBoxType::New();
  boundingBox->SetPoints(points);
  boundingBox->ComputeBoundingBox();
  BoundingBoxType::BoundsArrayType bounds = boundingBox->GetBounds();
  std::cout << "Bounds: " << bounds << std::endl;
  std::cout << "Center: " << boundingBox->GetCenter() << std::endl;

  const BoundingBoxType::PointsContainer* corners = boundingBox->GetCorners();

  VectorContainerType::ConstIterator corner = corners->Begin();
  while(corner != corners->End())
    {
    std::cout << corner->Value() << std::endl;
    corner++;
    }

  return EXIT_SUCCESS;
}
