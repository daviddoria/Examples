#include "/home/doriad/src/Insight/Code/Review/itkVTKPolyDataReader.h"

#include "itkMesh.h"

#include <string>
#include <iostream>

/* This can only read "legacy vtk binary (.vtk)" files, not legacy vtk ascii or .vtp files. */

int main( int argc, char ** argv )
{
	std::string InputFilename = argv[1];
	std::cout << "Input file: " << InputFilename << std::endl;
	
	typedef itk::Mesh<float, 3>                 MeshType;
	typedef itk::VTKPolyDataReader< MeshType >  ReaderType;

	ReaderType::Pointer  polyDataReader = ReaderType::New();

	typedef ReaderType::PointType   PointType;
	typedef ReaderType::VectorType  VectorType;

	polyDataReader->SetFileName(InputFilename.c_str());
		
	try
	{
		std::cerr << "Trying to read..." << std::endl;
		polyDataReader->Update();
	}
	catch( itk::ExceptionObject & excp )
	{
		std::cerr << "Error during Update() " << std::endl;
		std::cerr << excp << std::endl;
		return EXIT_FAILURE;
	}
	
	
	std::cout << "polyDataReader:" << std::endl <<
		polyDataReader << std::endl;

	MeshType::Pointer mesh = polyDataReader->GetOutput();

	PointType  point;

	std::cout << "Testing itk::VTKPolyDataReader" << std::endl;

	unsigned int numberOfPoints = mesh->GetNumberOfPoints();
	unsigned int numberOfCells  = mesh->GetNumberOfCells();

	std::cout << "numberOfPoints= " << numberOfPoints << std::endl;
	std::cout << "numberOfCells= " << numberOfCells << std::endl;

	if( !numberOfPoints )
	{
		std::cerr << "ERROR: numberOfPoints= " << numberOfPoints << std::endl;
		return EXIT_FAILURE;
	}

	if( !numberOfCells )
	{
		std::cerr << "ERROR: numberOfCells= " << numberOfCells << std::endl;
		return EXIT_FAILURE;
	}

 	//retrieve points
	for(unsigned int i = 0; i < numberOfPoints; i++)
	{
		PointType pp;
		bool pointExists = mesh->GetPoint(i, &pp);

		if(pointExists) 
		{
			std::cout << "Point is = " << pp << std::endl;
		}
	}

	return 0;
}