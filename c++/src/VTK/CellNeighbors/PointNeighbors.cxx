#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkIdList.h>
#include <vtkCell.h>
#include <vtkTriangleFilter.h>

#include <set>

int main(int, char *[])
{
  
  //Create a sphere
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();

  vtkSmartPointer<vtkTriangleFilter> triangleFilter = 
    vtkSmartPointer<vtkTriangleFilter>::New();
  triangleFilter->SetInputConnection(sphereSource->GetOutputPort());
  triangleFilter->Update();
  
  vtkIdType cellId = 0;
  
  vtkSmartPointer<vtkIdList> cellPointIds = 
    vtkSmartPointer<vtkIdList>::New();
  triangleFilter->GetOutput()->GetCellPoints(cellId, cellPointIds);
  
  std::set<unsigned int> neighbors;

  for(vtkIdType i = 0; i < cellPointIds->GetNumberOfIds(); i++)
    {
    vtkSmartPointer<vtkIdList> idList = 
      vtkSmartPointer<vtkIdList>::New();
    idList->InsertNextId(cellPointIds->GetId(i));
  
    //get the neighbors of the cell
    vtkSmartPointer<vtkIdList> neighborCellIds = 
      vtkSmartPointer<vtkIdList>::New();
      
    triangleFilter->GetOutput()->GetCellNeighbors(cellId, idList, neighborCellIds);
  
    for(vtkIdType j = 0; j < neighborCellIds->GetNumberOfIds(); j++)
      {
      neighbors.insert(neighborCellIds->GetId(j));
      }
      
    }

  std::cout << "Point neighbor ids are: " << std::endl;

  for(std::set<unsigned int>::iterator it1 = neighbors.begin(); it1 != neighbors.end(); it1++)
    {
    std::cout << " " << *it1;
    }

  return EXIT_SUCCESS;
}
