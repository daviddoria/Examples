#include "vtkPolyData.h"
#include "vtkXMLPolyDataWriter.h"
#include "vtkXMLPolyDataReader.h"
#include "vtkContourFilter.h"
#include "vtkMath.h"
#include "vtkGaussianSplatter.h"

#include <string>
#include <sstream>
#include <cstdlib>

int main(int argc, char **argv)
{
	if(argc != 4)
	{
		std::cout << "Required arguments: InputFilename OutputFilename radius\n";
		exit(-1);
	}
	std::string InputFilename, OutputFilename;
	InputFilename = argv[1];
	OutputFilename = argv[2];
	std::string strRadius = argv[3];
	std::stringstream ssRadius;
	ssRadius << strRadius;
	double radius;
	ssRadius >> radius;
	
	vtkXMLPolyDataReader* reader = vtkXMLPolyDataReader::New();
	reader->SetFileName(InputFilename.c_str());
	reader->Update();
	vtkPolyData* polydata = reader->GetOutput();
	/*
	vtkXMLPolyDataWriter* writer = vtkXMLPolyDataWriter::New();
	writer->SetFileName("points.vtp");
	writer->SetInput(polydata);
	writer->Write();
	*/
	vtkGaussianSplatter *Splatter = vtkGaussianSplatter::New();
	Splatter->SetInput(polydata);
	Splatter->SetSampleDimensions(50,50,50);
	Splatter->SetRadius(radius);
	Splatter->ScalarWarpingOff();

	vtkContourFilter *Surface = vtkContourFilter::New();
	Surface->SetInputConnection(Splatter->GetOutputPort());
	Surface->SetValue(0,0.01);

	vtkXMLPolyDataWriter* SurfaceWriter = vtkXMLPolyDataWriter::New();
	SurfaceWriter->SetFileName(OutputFilename.c_str());
	SurfaceWriter->SetInput(Surface->GetOutput());
	SurfaceWriter->Write();
	
	return 0;

}
