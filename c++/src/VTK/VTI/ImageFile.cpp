#include "vtkQuadric.h"
#include "vtkSampleFunction.h"
#include "vtkImageData.h"
#include "vtkXMLImageDataWriter.h"

int main ()
{
	// create the quadric function definition
	vtkQuadric *quadric = vtkQuadric::New();
	quadric->SetCoefficients(.5,1,.2,0,.1,0,0,.2,0,0);

	// sample the quadric function
	vtkSampleFunction *sample = vtkSampleFunction::New();
	sample->SetSampleDimensions(50,50,50);
	sample->SetImplicitFunction(quadric);
	double xmin = 0, xmax=1, ymin=0, ymax=1, zmin=0, zmax=1;
	sample->SetModelBounds(xmin, xmax, ymin, ymax, zmin, zmax);
	
	//the resulting volume has values corresponding to the distance from the quadratic surface
	vtkXMLImageDataWriter* writer = vtkXMLImageDataWriter::New();
	writer->SetInput(sample->GetOutput());
	writer->SetFileName("Test.vti");
	writer->Update();
	
	return 0;
}
