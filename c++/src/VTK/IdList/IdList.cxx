#include <vtkSmartPointer.h>
#include <vtkIdList.h>

int main(int, char*[])
{
  vtkSmartPointer<vtkIdList> idList1 = 
    vtkSmartPointer<vtkIdList>::New();
  idList1->InsertNextId(10);
  idList1->InsertNextId(20);
  
  std::cout << std::endl << "list 1: " << std::endl;
  for(vtkIdType i = 0; i < idList1->GetNumberOfIds(); i++)
    {
    std::cout << "Id " << i << " : " << idList1->GetId(i) << " ";
    }
    
  vtkSmartPointer<vtkIdList> idList2 = 
    vtkSmartPointer<vtkIdList>::New();
  idList2->InsertNextId(30);
  idList2->InsertNextId(40);
      
  std::cout << std::endl << "list 2: " << std::endl;
  for(vtkIdType i = 0; i < idList2->GetNumberOfIds(); i++)
    {
    std::cout << "Id " << i << " : " << idList2->GetId(i) << " ";
    }
    
    
  idList1->IntersectWith(idList2);
  std::cout << std::endl << "Combined lists: " << std::endl;
  for(vtkIdType i = 0; i < idList1->GetNumberOfIds(); i++)
    {
    std::cout << "Id " << i << " : " << idList1->GetId(i) << " ";
    }
    
  return EXIT_SUCCESS;
}
