#include "itkMesh.h"

int main(int, char *[])
{
  typedef   float   PixelType;

  const unsigned int Dimension = 3;
  typedef itk::Mesh< PixelType, Dimension >   MeshType;

  MeshType::Pointer  mesh = MeshType::New();

//create points
  MeshType::PointType p0;
  MeshType::PointType p1;
  MeshType::PointType p2;

  p0[0]= -1.0; p0[1]= -1.0; p0[2]= 0.0; // first  point ( -1, -1, 0 )
  p1[0]=  1.0; p1[1]= -1.0; p1[2]= 0.0; // second point (  1, -1, 0 )
  p2[0]=  1.0; p2[1]=  1.0; p2[2]= 0.0; // third  point (  1,  1, 0 )

  mesh->SetPoint( 0, p0 );
  mesh->SetPoint( 1, p1 );
  mesh->SetPoint( 2, p2 );

  std::cout << "Points = " << mesh->GetNumberOfPoints() << std::endl;

//access points
  typedef MeshType::PointsContainer::Iterator     PointsIterator;

  PointsIterator  pointIterator = mesh->GetPoints()->Begin();  

  PointsIterator end = mesh->GetPoints()->End();
  while( pointIterator != end ) 
    {
    MeshType::PointType p = pointIterator.Value();  // access the point
    std::cout << p << std::endl;                    // print the point
    ++pointIterator;                                // advance to next point
    }

  return 0;
}
