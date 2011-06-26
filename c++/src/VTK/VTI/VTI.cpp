#include <vtkIntArray.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPoints.h>
#include <vtkLine.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkImageData.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkAppendPolyData.h>
#include <vtkXMLImageDataReader.h>
#include <vtkXMLImageDataWriter.h>
#include <vtkPNGReader.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdlib.h>

int main(int argc, char *argv[])
{
	
	vtkPNGReader* reader = vtkPNGReader::New();
	reader->SetFileName("Test.png");
	reader->Update();
	
	vtkImageData* imagedata = reader->GetOutput();
	
	vtkXMLImageDataWriter* writer = vtkXMLImageDataWriter::New();
	writer->SetInput(imagedata);
	writer->SetFileName("Test.vti");
	writer->Update();
		
	
	return 0;
}
