#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkIdList.h>
#include <vtkKdTree.h>
#include <vtkVertexGlyphFilter.h>

int main(int argc, char *argv[])
{
  //Setup point coordinates
  double x[3] = {1.0, 0.0, 0.0};
  double y[3] = {0.0, 1.0, 0.0};
  double z[3] = {0.0, 0.0, 1.0};

  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(x);
  points->InsertNextPoint(y);
  points->InsertNextPoint(z);
  
  //Create the tree
  vtkSmartPointer<vtkKdTree> kDTree = 
      vtkSmartPointer<vtkKdTree>::New();
  kDTree->BuildLocatorFromPoints(points);
  
  double testPoint[3] = {2.0, 0.0, 0.0};
  
  //Find the closest points to TestPoint
  double closestPointDist;
  vtkIdType id = kDTree->FindClosestPoint(testPoint, closestPointDist); //vtkKdTree::FindClosestPoint: must build locator first
  cout << "The closest point is point " << id << endl;
  
  return EXIT_SUCCESS;
}
