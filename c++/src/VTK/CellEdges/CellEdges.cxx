#include <vtkSmartPointer.h>
#include <vtkTriangle.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkIdList.h>

int main(int argc, char *argv[])
{
  
  vtkSmartPointer<vtkTriangle> triangle = 
      vtkSmartPointer<vtkTriangle>::New();
  triangle->GetPoints()->SetPoint(0, 1.0, 0.0, 0.0);
  triangle->GetPoints()->SetPoint(1, 0.0, 0.0, 0.0);
  triangle->GetPoints()->SetPoint(2, 0.0, 1.0, 0.0);
  triangle->GetPointIds()->SetId ( 0, 0 );
  triangle->GetPointIds()->SetId ( 1, 1 );
  triangle->GetPointIds()->SetId ( 2, 2 );
  
  cout << "The cell has " << triangle->GetNumberOfEdges() << " edges." << endl;
  
  for(vtkIdType i = 0; i < triangle->GetNumberOfEdges(); i++)
    {
    vtkCell* edge = triangle->GetEdge(i);

    vtkIdList* pointIdList = edge->GetPointIds();
    cout << "Edge " << i << " has " << pointIdList->GetNumberOfIds() << " points."  << endl;
    
    for(vtkIdType p = 0; p < pointIdList->GetNumberOfIds(); p++)
      {
      cout << "Edge " << i << " uses point " << pointIdList->GetId(p) << endl;
      }
      
    }
  
  
  return EXIT_SUCCESS;
}
