#include "vtkPolyData.h"
#include "vtkXMLPolyDataWriter.h"
#include "vtkContourFilter.h"
#include "vtkMath.h"
#include "vtkGaussianSplatter.h"

#include <cmath>

vtkPoints* CreatePoints();

int main(int argc, char **argv)
{

	// Create points on a sphere
	vtkPoints* points = CreatePoints();
	vtkPolyData* polydata = vtkPolyData::New();
	polydata->SetPoints(points);

	vtkXMLPolyDataWriter* writer = vtkXMLPolyDataWriter::New();
	writer->SetFileName("points.vtp");
	writer->SetInput(polydata);
	writer->Write();
	
	vtkGaussianSplatter *Splatter = vtkGaussianSplatter::New();
	Splatter->SetInput(polydata);
	Splatter->SetSampleDimensions(50,50,50);
	Splatter->SetRadius(0.5);
	Splatter->ScalarWarpingOff();

	vtkContourFilter *Surface = vtkContourFilter::New();
	Surface->SetInputConnection(Splatter->GetOutputPort());
	Surface->SetValue(0,0.01);

	vtkXMLPolyDataWriter* SurfaceWriter = vtkXMLPolyDataWriter::New();
	SurfaceWriter->SetFileName("surface.vtp");
	SurfaceWriter->SetInput(Surface->GetOutput());
	SurfaceWriter->Write();
	
	return 0;

}

vtkPoints* CreatePoints()
{
	vtkPoints* points = vtkPoints::New();
	
	float x, y, z;
	// generate random points on unit sphere
	for(unsigned int i = 0; i < 100; i++) 
	{
		double phi, theta,u,v;
		u = vtkMath::Random(0.0,1.0);
		v = vtkMath::Random(0.0,1.0);
		phi = 2.0*3.14159265*u;
		theta = acos(2.0*v-1.0);

		x = std::cos(phi)*std::sin(theta);
		y = std::sin(phi)*std::sin(theta);
		z = std::cos(theta);


		points->InsertNextPoint(x, y, z);
	}
	return points;
}
