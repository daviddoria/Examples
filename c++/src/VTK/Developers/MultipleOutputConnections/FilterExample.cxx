#include <vtkSmartPointer.h>
#include "vtkTestFilter.h"

int main(int, char*[])
{
  //setup the second input
  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(4.0, 5.0, 6.0);
  vtkSmartPointer<vtkPolyData> polydata =
    vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);

  vtkSmartPointer<vtkTestFilter> filter =
    vtkSmartPointer<vtkTestFilter>::New();
  filter->SetInputConnection(polydata->GetProducerPort());
  filter->Update();

  vtkPolyData* output0 = filter->GetOutput(0);
  vtkPolyData* output1 = filter->GetOutput(1);

  std::cout << "Output0 has " << output0->GetNumberOfPoints() << " points." << std::endl;
  std::cout << "Output1 has " << output1->GetNumberOfPoints() << " points." << std::endl;

  return EXIT_SUCCESS;
}
