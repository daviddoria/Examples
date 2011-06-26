#include <vtkSmartPointer.h>
#include <vtkShepardMethod.h>
#include <vtkContourFilter.h>
#include <vtkFloatArray.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0.0, 0.0, 0.0);
  points->InsertNextPoint(1.0, 0.0, 0.0);
  points->InsertNextPoint(2.3, 0.0, 0.0);
      
  vtkSmartPointer<vtkFloatArray> distances = 
      vtkSmartPointer<vtkFloatArray>::New();
  distances->SetNumberOfComponents(1);
  distances->SetName("Distances");
  
  distances->InsertNextValue(1.0);
  distances->InsertNextValue(2.0);
  distances->InsertNextValue(3.0);
  
  vtkSmartPointer<vtkPolyData> polydata = 
      vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
  polydata->GetPointData()->SetScalars(distances);
  
  vtkSmartPointer<vtkShepardMethod> shepard = 
      vtkSmartPointer<vtkShepardMethod>::New();
  shepard->SetInput(polydata);
  shepard->Update();
  
  vtkSmartPointer<vtkContourFilter> contourFilter = 
      vtkSmartPointer<vtkContourFilter>::New();
  contourFilter->SetInput(shepard->GetOutput());
  contourFilter->Update();
      
  return 0;
}
