#include <vtkSmartPointer.h>
#include <vtkSmartPointer.h>
#include <vtkNew.h>
#include <vtkPolyData.h>

int main(int, char *[])
{
//   vtkNew<vtkPoints> points;
//   vtkNew<vtkPolyData> polydata;
//   polydata->SetPoints(points.GetPointer());
  
  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkPolyData> polydata =
    vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);

  return EXIT_SUCCESS;
}
