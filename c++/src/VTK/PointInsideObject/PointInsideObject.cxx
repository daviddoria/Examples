#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkSmartPointer.h>
#include <vtkDelaunay3D.h>
#include <vtkUnstructuredGrid.h>

int main(int argc, char **argv)
{
  vtkSmartPointer<vtkSphereSource> sphereSource =
    vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetCenter(0.0, 0.0, 0.0);
  sphereSource->SetRadius(1.0);
  sphereSource->Update();

  vtkSmartPointer<vtkDelaunay3D> sphere = 
      vtkSmartPointer<vtkDelaunay3D>::New();
  sphere->SetInput(sphereSource->GetOutput());
  sphere->Update();

  double testInside[3] = {.5, 0.0, 0.0};
  double testOutside[3] = {10.0, 0.0, 0.0};

  double pcoords[3], weights[3];

  vtkIdType cellId;

  int subId;

 //should be inside
  cellId = sphere->GetOutput()->FindCell(testInside, NULL, 0, .1,
                            subId, pcoords, weights);
  if (cellId >= 0)
    {
    cout << "In cell " << cellId << endl;
    cout << "inside" << endl;
    }
  else
    {
    cout << "outside" << endl;
    }

 //should be outside
  cellId = sphere->GetOutput()->FindCell(testOutside, NULL, 0, 0.1,
                            subId, pcoords, weights);
  if (cellId >= 0)
    {
    cout << "In cell " << cellId << endl;
    cout << "inside" << endl;
    }
  else
    {
    cout << "outside" << endl;
    }

  return EXIT_SUCCESS;
}