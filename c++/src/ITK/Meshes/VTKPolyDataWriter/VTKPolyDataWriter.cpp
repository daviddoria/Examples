#include "itkMesh.h"
#include "itkLineCell.h"
#include "itkVTKPolyDataWriter.h"

const unsigned int Dimension = 3;
typedef itk::Mesh< float, Dimension >   MeshType;
	
MeshType::Pointer CreateMeshWithEdges();

int main(int, char *[])
{

  MeshType::Pointer mesh = CreateMeshWithEdges();

  typedef itk::VTKPolyDataWriter<MeshType> WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput(mesh);
  writer->SetFileName("test.vtk");
  writer->Update();
  
  return EXIT_SUCCESS;
}


MeshType::Pointer CreateMeshWithEdges()
{

  MeshType::Pointer  mesh = MeshType::New();

  // Create points
  MeshType::PointType p0,p1,p2,p3;

  p0[0]= -1.0; p0[1]= -1.0; p0[2]= 0.0; // first  point ( -1, -1, 0 )
  p1[0]=  1.0; p1[1]= -1.0; p1[2]= 0.0; // second point (  1, -1, 0 )
  p2[0]=  1.0; p2[1]=  1.0; p2[2]= 0.0; // third  point (  1,  1, 0 )
  p3[0]=  1.0; p3[1]=  1.0; p3[2]= 1.0; // third  point (  1,  1, 1 )

  mesh->SetPoint( 0, p0 );
  mesh->SetPoint( 1, p1 );
  mesh->SetPoint( 2, p2 );
  mesh->SetPoint( 3, p3 );

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

  typedef MeshType::CellType::CellAutoPointer         CellAutoPointer;
  typedef itk::LineCell< MeshType::CellType >         LineType;


  CellAutoPointer line0;
  line0.TakeOwnership(  new LineType  );
  line0->SetPointId(0, 0); // line between points 0 and 1
  line0->SetPointId(1, 1);
  mesh->SetCell( 0, line0);

  CellAutoPointer line1;
  line1.TakeOwnership(  new LineType  );
  line1->SetPointId(0, 1); // line between points 1 and 2
  line1->SetPointId(1, 2);
  mesh->SetCell( 1, line1);

  CellAutoPointer line2;
  line2.TakeOwnership(  new LineType  );
  line2->SetPointId(0, 2); // line between points 2 and 3
  line2->SetPointId(1, 3);
  mesh->SetCell( 2, line2);

  return mesh;
}
