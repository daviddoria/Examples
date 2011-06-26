#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPointSource.h>

#include "vtkTestFilter.h"

int main (int argc, char *argv[])
{
  vtkSmartPointer<vtkPointSource> pointSource = 
      vtkSmartPointer<vtkPointSource>::New();
  pointSource->Update();
  
  vtkSmartPointer<vtkPolyData> points = pointSource->GetOutput();
  cout << "points is a : " << points->GetClassName() << endl; 
  
  vtkSmartPointer<vtkTestFilter> filter = 
      vtkSmartPointer<vtkTestFilter>::New();
  filter->SetInput(points);
  filter->Update();
  
  vtkPointSet* output = filter->GetOutput();
  cout << "output is a : " << output->GetClassName() << endl; 
  //cout << "Output points: " << outputPolydata->GetNumberOfPoints() << vtkstd::endl;
  
  return EXIT_SUCCESS;
}
