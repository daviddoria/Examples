#include "itkMesh.h"

#include "itkDistanceThresholdMeshFilter.h"

#include <cstdlib>
#include <iostream>

typedef   float   PixelType;

const unsigned int Dimension = 3;
typedef itk::Mesh< PixelType, Dimension >   MeshType;

MeshType::Pointer CreateMesh();
void WritePoints(MeshType::Pointer mesh);

int main( int argc, char *argv[])
{
	MeshType::Pointer mesh = CreateMesh();
	WritePoints(mesh);

	typedef itk::DistanceThresholdMeshFilter < MeshType, MeshType> DistanceFilterType;
	DistanceFilterType::Pointer distanceThreshold = DistanceFilterType::New();
	/*
	float lower = 95.0;
	float upper = 105.0;
	connectedThreshold->SetLower(lower);
	connectedThreshold->SetUpper(upper);
	
	connectedThreshold->SetReplaceValue(255);
	
	//seed 1: (25, 35)
	ImageType::IndexType seed1;
  
	seed1[0] = 25;
	seed1[1] = 35;
	connectedThreshold->SetSeed(seed1);
		
	//setup pipeline
	connectedThreshold->SetInput(image);
	caster->SetInput(connectedThreshold->GetOutput() );
	writer->SetInput(caster->GetOutput());
	writer->SetFileName( "region1.png" );
	writer->Update();
	
	//seed 2: (110, 120)
	ImageType::IndexType seed2;
	seed2[0] = 110;
	seed2[1] = 120;
	connectedThreshold->SetSeed(seed2);
	connectedThreshold->SetReplaceValue(150);
		
	//setup pipeline
	connectedThreshold->SetInput(image);
	caster->SetInput(connectedThreshold->GetOutput() );
	writer->SetInput(caster->GetOutput());
	writer->SetFileName( "region2.png" );
	writer->Update();
	*/
	
	return 0;
}

MeshType::Pointer CreateMesh()
{


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

	//std::cout << "Points = " << mesh->GetNumberOfPoints() << std::endl;

	return mesh;
}

void WritePoints(MeshType::Pointer mesh)
{
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
}