#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPointSet.h>

int main()
{
  //create a set of points
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
  Points->InsertNextPoint ( 1.0, 0.0, 0.0 );
  Points->InsertNextPoint ( 0.0, 0.0, 0.0 );
  Points->InsertNextPoint ( 0.0, 1.0, 0.0 );

  //create a point set
  vtkSmartPointer<vtkPointSet> PointSet = vtkSmartPointer<vtkPointSet>::New();

  //add the points to the polydata
 // PointSet->SetPoints ( Points );

  return 0;
}
