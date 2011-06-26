#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkIdList.h>
#include <vtkModifiedBSPTree.h>
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
  vtkSmartPointer<vtkModifiedBSPTree> tree =
      vtkSmartPointer<vtkModifiedBSPTree>::New();
  tree->SetDataSet(glyphFilter->GetOutput());
  tree->BuildLocator();
 
  double testPoint[3] = {2.0, 0.0, 0.0};

  /*
  //DOES NOT YET SUPPORT FindClosestPoint
  
  //Find the closest points to TestPoint
  double closestPointDist;
  double closestPoint[3];
  vtkIdType cellId; int subId; //not used
  tree->FindClosestPoint(testPoint, closestPoint, cellId, subId, closestPointDist);
  std::cout << "The closest point is (" << closestPoint[0] << " , " << closestPoint[1] << " , "
            << closestPoint[2] << ") with distance " << closestPointDist << std::endl;

  */
  
  return EXIT_SUCCESS;
}