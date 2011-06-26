#include "itkQuadEdgeMesh.h"

#include "itkLineCell.h"

const unsigned int Dimension = 3;
typedef itk::QuadEdgeMesh< float, Dimension >   MeshType;
	

int main(int, char *[])
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

	std::cout << "NumPoints = " << mesh->GetNumberOfPoints() << std::endl;

	//create an edge between points 2 and 3
	unsigned int pointId0 = 2;
	unsigned int pointId1 = 3;
	mesh->AddEdge(pointId0, pointId1);

	//access edges
	
	MeshType::QEPrimal* edge = mesh->GetEdge();
	
	MeshType::CoordRepType origin = edge->GetOrigin();
	MeshType::CoordRepType destination = edge->GetDestination();
	std::cout << "origin: " << origin << std::endl;
	std::cout << "Destination: " << destination << std::endl;
	
	//retrieve the coordinate of the point at the origin of the edge we selected
	
	//can't do this (segfault))
	//MeshType::PointType::Pointer p = MeshType::PointType::New() ;
	//MeshType::PointType* p = MeshType::PointType::New() ;
	//mesh->GetPoint(origin, p); 
	
	MeshType::PointType p;
	mesh->GetPoint(origin, &p);
	
	std::cout << "p: " << p[0] << " " << p[1] << " " << p[2] << std::endl;

	return 0;
}

/* access points
	typedef MeshType::PointsContainer::Iterator     PointsIterator;

	PointsIterator  pointIterator = mesh->GetPoints()->Begin();  

	PointsIterator end = mesh->GetPoints()->End();
	while( pointIterator != end ) 
{
		MeshType::PointType p = pointIterator.Value();  // access the point
		std::cout << p << std::endl;                    // print the point
		++pointIterator;                                // advance to next point
}
*/

/* access cells

	typedef MeshType::CellsContainer::Iterator  CellIterator;
	CellIterator  cellIterator = mesh->GetCells()->Begin();
	CellIterator  CellsEnd          = mesh->GetCells()->End();
	
	while( cellIterator != CellsEnd ) 
{
		MeshType::CellType* cellptr = cellIterator.Value();
		LineType* line = dynamic_cast<LineType *>( cellptr );

		long unsigned int* linePoint0 = line->PointIdsBegin();
		//long unsigned int* linePoint1 = line->PointIdsEnd();
		long unsigned int* linePoint1 = linePoint0+1;
		std::cout << "line first point id: " << *linePoint0 << std::endl;
		std::cout << "line second point id: " << *linePoint1 << std::endl;
		
	
		++cellIterator;
}

*/