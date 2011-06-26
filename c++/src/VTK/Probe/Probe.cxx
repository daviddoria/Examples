#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkProbeFilter.h>
#include <vtkXMLPolyDataWriter.h>

int main(int argc, char *argv[])
{
	//create points
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
	
	Points->InsertNextPoint(1.0, 0.0, 0.0);
	Points->InsertNextPoint(0.0, 1.0, 0.0);
	Points->InsertNextPoint(0.0, 0.0, 1.0);
	
	//add the points to a polydata
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(Points);
		
    vtkSmartPointer<vtkPoints> ProbePoints = vtkSmartPointer<vtkPoints>::New();
    ProbePoints->InsertNextPoint(0.0, 0.0, 0.0);
    vtkSmartPointer<vtkPolyData> ProbePolydata = vtkSmartPointer<vtkPolyData>::New();
    ProbePolydata->SetPoints(ProbePoints);
    
    vtkSmartPointer<vtkProbeFilter> ProbeFilter = vtkSmartPointer<vtkProbeFilter>::New();
    ProbeFilter->SetSource(polydata);
    ProbeFilter->SetInput(ProbePolydata);
    ProbeFilter->Update();
    
    vtkSmartPointer<vtkXMLPolyDataWriter> Writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    Writer->SetFileName("output.vtp");
    Writer->SetInput(ProbeFilter->GetOutput());
    Writer->Write();
    
	return 0;
}

