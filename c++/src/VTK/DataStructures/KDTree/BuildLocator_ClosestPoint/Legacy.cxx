#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkIdList.h>
#include <vtkKdTree.h>
#include <vtkDataSetCollection.h>
#include <vtkVertexGlyphFilter.h>
 
int main(int argc, char *argv[])
{
  //Setup point coordinates
  double x[3] = {1.0, 0.0, 0.0};
  double y[3] = {0.0, 1.0, 0.0};
  double z[3] = {0.0, 0.0, 1.0};
 
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint (x);
  points->InsertNextPoint (y);
  points->InsertNextPoint (z);
  
  vtkSmartPointer<vtkPolyData> polydata = 
      vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
  
  //the tree needs cells, so add vertices to each point
  vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = 
      vtkSmartPointer<vtkVertexGlyphFilter>::New();
  vertexFilter->SetInputConnection(polydata->GetProducerPort());
  vertexFilter->Update();
  
  //Create the tree
  vtkSmartPointer<vtkKdTree> kDTree = 
      vtkSmartPointer<vtkKdTree>::New();
  kDTree->AddDataSet(vertexFilter->GetOutput());
  kDTree->SetMinCells(1);
  kDTree->BuildLocator();
  
  double testPoint[3] = {.7, 0.0, 0.0};
  double dist;
  
  vtkSmartPointer<vtkIdList> closestPointIds = 
      vtkSmartPointer<vtkIdList>::New();
  
  kDTree->FindClosestNPoints(1, testPoint, closestPointIds);
  
  return EXIT_SUCCESS;
}
