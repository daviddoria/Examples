#include <iostream>
#include <string>
#include <sstream>

#include "/home/doriad/src/Insight/Code/Review/itkVTKPolyDataReader.h"
#include "/home/doriad/src/Insight/Code/Review/itkVTKPolyDataWriter.h"
#include "itkQuadEdgeMesh.h"

#include "itkQuadEdgeMeshDecimationCriteria.h"
#include "itkQuadEdgeMeshSquaredEdgeLengthDecimation.h"

const unsigned int Dimension = 3;
typedef itk::QuadEdgeMesh< float, Dimension >   MeshType;

int main( int argc, char ** argv )
{
	if(argc != 4)
	{
		std:: cout << "Required arguments: InputFilename OutputFilename" << std::endl;
		exit(-1);
	}
	
	std::string InputFilename = argv[1];
	std::cout << "Input file: " << InputFilename << std::endl;
	
	std::string OutputFilename = argv[2];
	std::cout << "Output file: " << OutputFilename << std::endl;
	
	std::string strTriangleCount = argv[3];
	std::stringstream ssTriangleCount;
	ssTriangleCount << strTriangleCount;
	unsigned int TriangleCount;
	ssTriangleCount >> TriangleCount;
	
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

	typedef itk::NumberOfFacesCriterion<MeshType> CriterionType;
	CriterionType::Pointer criterion = CriterionType::New();
	criterion->SetTopologicalChange(false);
	criterion->SetNumberOfElements(TriangleCount);
	
	typedef itk::QuadEdgeMeshSquaredEdgeLengthDecimation<MeshType,MeshType,CriterionType> DecimationType;
	
	DecimationType::Pointer decimate = DecimationType::New();
	decimate->SetInput(mesh);
	decimate->SetCriterion(criterion);
//	decimate->SetRelocate(true); //where did this go?
	decimate->Update();
	
	MeshType::Pointer OutputMesh = decimate->GetOutput();
	
	{
	typedef itk::VTKPolyDataWriter< MeshType >  WriterType;

	WriterType::Pointer polyDataWriter = WriterType::New();

	polyDataWriter->SetFileName(OutputFilename.c_str());
	polyDataWriter->SetInput(OutputMesh);
	polyDataWriter->Update();
	polyDataWriter->Write();
	}
	
	return 0;
}