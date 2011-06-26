#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkLine.h>
#include <vtkOBBTree.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  //create the locator
  vtkSmartPointer<vtkOBBTree> tree = 
      vtkSmartPointer<vtkOBBTree>::New();
  tree->SetDataSet(sphereSource->GetOutput());
  tree->BuildLocator();
  
  //intersect the locator with the line
  double lineP0[3] = {0.0, 0.0, 0.0};
  double lineP1[3] = {0.0, 0.0, 2.0};
  vtkSmartPointer<vtkPoints> intersectPoints = 
      vtkSmartPointer<vtkPoints>::New();
  
  tree->IntersectWithLine(lineP0, lineP1, intersectPoints, NULL);

  double intersection[3];
  intersectPoints->GetPoint(0, intersection);
  cout << "NumPoints: " << intersectPoints->GetNumberOfPoints() << endl;
  cout << "Intersection: " << intersection[0] << ", " << intersection[1] << ", " << intersection[2] << endl;
  
  return EXIT_SUCCESS;
}
