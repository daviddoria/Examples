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
	
	//duplicate points
	//p0[0]= -1.0; p0[1]= -1.0; p0[2]= -1.0; // first  point ( -1, -1, 0 )
	//p1[0]= -1.0; p1[1]= -1.0; p1[2]= -1.0; // first  point ( -1, -1, 0 )

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

	//mesh->AddEdge(0,1);
	//std::cout << "Edges = " << mesh->GetNumberOfEdges() << std::endl;
	
	//separate edges - 2 edges are actually added
	/*
	mesh->AddEdge(0,1);
	mesh->AddEdge(1,2);
	std::cout << "NumEdges = " << mesh->GetNumberOfEdges() << std::endl;
	*/
	
	//duplicate edges - only 1 edge is really added
	bool e1, e2;
	e1 = mesh->AddEdge(0,1);
	e2 = mesh->AddEdge(0,1);
	std::cout << "Edges = " << mesh->GetNumberOfEdges() << std::endl;
	std::cout << "e1 successful? " << e1 << std::endl;
	std::cout << "e2 successful? " << e2 << std::endl;
	
	//reverse edges - again, only 1 edge is really added
	/*
	mesh->AddEdge(0,1);
	mesh->AddEdge(1,0);
	std::cout << "Edges = " << mesh->GetNumberOfEdges() << std::endl;
	*/
	
	//invalid edge - an edge is attempted to be added on invalid points - 0 edges are actually added
	/*
	mesh->AddEdge(7,8);
	std::cout << "Edges = " << mesh->GetNumberOfEdges() << std::endl;
	*/
	
	//cells
	
		/*
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
		MeshType::CellType* cellptr = cellIterator.Value();
		LineType* line = dynamic_cast<LineType *>( cellptr );

		long unsigned int* linePoint0 = line->PointIdsBegin();
		//long unsigned int* linePoint1 = line->PointIdsEnd();
		long unsigned int* linePoint1 = linePoint0+1;
		std::cout << "line first point id: " << *linePoint0 << std::endl;
		std::cout << "line second point id: " << *linePoint1 << std::endl;
		
	
		++cellIterator;
	}
	
	//access edges
	
	MeshType::QEPrimal* edge = mesh->GetEdge();
	
	MeshType::CoordRepType origin = edge->GetOrigin();
	MeshType::CoordRepType destination = edge->GetDestination();
	std::cout << "origin: " << origin << std::endl;
	std::cout << "Destination: " << destination << std::endl;
	
	MeshType::PointType p;
	//mesh->GetPoint(origin, p);
	//mesh->GetPoint(2, p);
	//std::cout << "p: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
*/
	return 0;
}

