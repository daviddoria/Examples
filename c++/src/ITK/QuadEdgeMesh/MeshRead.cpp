
#include <string>
#include <iostream>

#include "/home/doriad/src/Insight/Code/Review/itkVTKPolyDataReader.h"
//#include "itkQuadEdgeMesh.h"
#include "itkMesh.h"

const unsigned int Dimension = 3;
//typedef itk::QuadEdgeMesh< float, Dimension >   MeshType;
typedef itk::Mesh< float, Dimension >   MeshType;

/* This can only read "legacy vtk ascii (.vtk)" files, not legacy vtk binary or .vtp files. */

int main( int argc, char ** argv )
{
	if(argc != 2)
	{
		std:: cout << "Required arguments: Filename" << std::endl;
		exit(-1);
	}
	
	std::string InputFilename = argv[1];
	std::cout << "Input file: " << InputFilename << std::endl;
	
	typedef itk::VTKPolyDataReader< MeshType >  ReaderType;

	ReaderType::Pointer polyDataReader = ReaderType::New();

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


	return 0;
}