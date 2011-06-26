#include "itkBlobSpatialObject.h"

int main( int argc, char *argv[] )
{
  typedef itk::BlobSpatialObject<2> BlobType;

  // Create a list of points
  BlobType::PointListType points;
  for(unsigned int i = 0; i < 20; i++)
    {
    BlobType::BlobPointType point;
    point.SetPosition(i,i);

    points.push_back(point);
    }

  BlobType::Pointer blob = BlobType::New();
  blob->SetPoints(points);

  BlobType::BoundingBoxType::BoundsArrayType bounds = blob->GetBoundingBox()->GetBounds();
  std::cout << "Bounds: " << bounds << std::endl;

  return EXIT_SUCCESS;
}
