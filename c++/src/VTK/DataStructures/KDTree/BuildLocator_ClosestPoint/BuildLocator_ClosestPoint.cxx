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
 
  vtkSmartPointer<vtkPolyData> polydata = 
      vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
 
  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = 
      vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputConnection(polydata->GetProducerPort());
  glyphFilter->Update();
 
  //Create the tree
  vtkSmartPointer<vtkKdTree> kDTree = 
      vtkSmartPointer<vtkKdTree>::New();
  //kDTree->SetDataSet(polydata);
  kDTree->SetDataSet(glyphFilter->GetOutput());
  kDTree->BuildLocator();
 
  double testPoint[3] = {2.0, 0.0, 0.0};
 
  vtkSmartPointer<vtkIdList> idList = 
      vtkSmartPointer<vtkIdList>::New();
  kDTree->FindPointsWithinRadius(1.0, testPoint, idList);
  /*
  //Find the closest points to TestPoint
  double closestPointDist;
  vtkIdType id = kDTree->FindClosestPoint(testPoint, closestPointDist); //vtkKdTree::FindClosestPoint: must build locator first
  cout << "The closest point is point " << id << endl;
 
  //Get the coordinates of the closest point
  double closestPoint[3];
  kDTree->GetDataSet(0)->GetPoint(id, closestPoint);
  cout << "Coordinates: " << closestPoint[0] << " " << closestPoint[1] << " " << closestPoint[2] << endl;
 */
  
  return EXIT_SUCCESS;
}