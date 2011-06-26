#include <vtkSmartPointer.h>

#include "vtkTestFilter.h"

int main (int argc, char *argv[])
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0.0, 0.0, 0.0);
  
  vtkSmartPointer<vtkPolyData> inputPolydata = vtkSmartPointer<vtkPolyData>::New();
  inputPolydata->SetPoints(points);
  
  vtkstd::cout << "Input points: " << inputPolydata->GetNumberOfPoints() << vtkstd::endl;
  
  vtkSmartPointer<vtkTestFilter> filter = vtkSmartPointer<vtkTestFilter>::New();
  filter->SetInput(inputPolydata);
  filter->Update();
  
  vtkPolyData* outputPolydata = filter->GetOutput();
  
  vtkstd::cout << "Output points: " << outputPolydata->GetNumberOfPoints() << vtkstd::endl;
  
  return 0;
}
