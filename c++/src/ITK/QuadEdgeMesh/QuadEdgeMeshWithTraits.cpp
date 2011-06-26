#include "itkQuadEdgeMesh.h"
#include "itkQuadEdgeMeshTraits.h"

#include "itkMesh.h"
#include "itkLineCell.h"

const unsigned int Dimension = 3;

//QE with traits
typedef itk::QuadEdgeMeshTraits<bool, 3, double,double> MeshTraits;
typedef itk::QuadEdgeMesh<bool, 3, MeshTraits > MeshType;


//QE without traits
/*
typedef itk::QuadEdgeMesh< float, Dimension >   MeshType;
*/

//Standard mesh with traits
/*
typedef itk::DefaultStaticMeshTraits<bool, 3, 2,
				double,double> MeshTraits;
typedef itk::Mesh< double, 3, MeshTraits > MeshType;
*/

MeshType::Pointer CreatePointOnlyMesh();
void CreateMeshWithEdges();

int main(int, char *[])
{
	CreatePointOnlyMesh();
	CreateMeshWithEdges();
 
  return 0;
}


MeshType::Pointer CreatePointOnlyMesh()
{
	MeshType::Pointer  mesh = MeshType::New();

//create points
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

	return mesh;
}



void CreateMeshWithEdges()
{
	MeshType::Pointer mesh = CreatePointOnlyMesh();
	
	typedef MeshType::CellType::CellAutoPointer         CellAutoPointer;
	typedef itk::LineCell< MeshType::CellType >         LineType;
	//create a link to the previous point in the column (below the current point)
	CellAutoPointer colline;
	colline.TakeOwnership(  new LineType  );
				
	//unsigned int pointId0 = 0;
	//unsigned int pointId1 = 1;
	
	unsigned int pointId0 = 2;
	unsigned int pointId1 = 3;
	
	colline->SetPointId(0, pointId0); // line between points 0 and 1
	colline->SetPointId(1, pointId1);
	//std::cout << "Linked point: " << MeshIndex << " and " << MeshIndex - 1 << std::endl;
	mesh->SetCell( 0, colline );

	typedef MeshType::CellsContainer::Iterator  CellIterator;
	CellIterator  cellIterator = mesh->GetCells()->Begin();
	CellIterator  CellsEnd          = mesh->GetCells()->End();
	
	while( cellIterator != CellsEnd ) 
	{
		MeshType::CellType * cellptr = cellIterator.Value();
		LineType * line = dynamic_cast<LineType *>( cellptr );

		long unsigned int* linePoint0 = line->PointIdsBegin();
		//long unsigned int* linePoint1 = line->PointIdsEnd();
		long unsigned int* linePoint1 = linePoint0+1;
		std::cout << "line first point id: " << *linePoint0 << std::endl;
		std::cout << "line second point id: " << *linePoint1 << std::endl;
		
		++cellIterator;
	}
	
}
