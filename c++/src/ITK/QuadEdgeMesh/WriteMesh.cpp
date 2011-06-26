#include "itkQuadEdgeMesh.h"
#include "itkLineCell.h"
#include "/home/doriad/src/Insight/Code/Review/itkVTKPolyDataWriter.h"

#include <string>

const unsigned int Dimension = 3;
typedef itk::QuadEdgeMesh< float, Dimension >   MeshType;
	

int main(int argc, char * argv[])
{
	if(argc != 2)
	{
		std::cout << "Required arguments: Filename" << std::endl;
		exit(-1);
	}
	
	std::string Filename = argv[1];
	
	MeshType::Pointer  mesh = MeshType::New();

	//create points
	MeshType::PointType p0,p1,p2,p3;
	
	p0[0] = -1.0; p0[1]= -1.0; p0[2]= 0.0; // first  point ( -1, -1, 0 )
	p1[0] =  1.0; p1[1]= -1.0; p1[2]= 0.0; // second point (  1, -1, 0 )
	p2[0] =  1.0; p2[1]=  1.0; p2[2]= 0.0; // third  point (  1,  1, 0 )
	p3[0] =  1.0; p3[1]=  1.0; p3[2]= 1.0; // third  point (  1,  1, 1 )

	mesh->SetPoint( 0, p0 );
	mesh->SetPoint( 1, p1 );
	mesh->SetPoint( 2, p2 );
	mesh->SetPoint( 3, p3 );

	std::cout << "NumPoints = " << mesh->GetNumberOfPoints() << std::endl;

	//create 2 edges
	mesh->AddEdge(0, 1);
	mesh->AddEdge(1, 2);

	std::cout << "NumEdges = " << mesh->GetNumberOfEdges() << std::endl;
	
	typedef itk::VTKPolyDataWriter< MeshType >  WriterType;
	WriterType::Pointer SubmeshWriter = WriterType::New();

	SubmeshWriter->SetFileName(Filename);
	SubmeshWriter->SetInput(mesh);
	SubmeshWriter->Update();
	SubmeshWriter->Write();
	
	return 0;
}
 
