#include "vtkLidarPoint.h"
#include "vtkRay.h"

#include "vtkSmartPointer.h"

#include <vector>

int main()
{
 
  vtkstd::vector<vtkstd::vector<vtkSmartPointer<vtkLidarPoint> > > Grid;
  Grid.resize(10);
  
  for(unsigned int i = 0; i < 2; i++)
    {
    vtkstd::vector<vtkSmartPointer<vtkLidarPoint> > Column;
    Column.clear();
    Column.resize(10);
  
    for(unsigned int j = 0; j < 2; j++)
      {
      vtkSmartPointer<vtkRay> Ray = vtkSmartPointer<vtkRay>::New();
      
      vtkSmartPointer<vtkLidarPoint> LidarPoint = vtkSmartPointer<vtkLidarPoint>::New();
      vtkstd::cout << "After LidarPoint::New(): " << LidarPoint->GetReferenceCount() << vtkstd::endl;
      LidarPoint->SetRay(Ray);
      LidarPoint->DebugOn();
      Column[j] = LidarPoint;
      vtkstd::cout << "After Column[" << j << "]=LidarPoint : " << LidarPoint->GetReferenceCount() << vtkstd::endl;
      
      }
    std::cout << "-------------- End Of for(j) ---------------" << std::endl;      
    vtkstd::cout << "After leave for(j) scope, Column[0]: " << Column[0]->GetReferenceCount() << vtkstd::endl;    
    vtkstd::cout << "After leave for(j) scope, Column[1]: " << Column[0]->GetReferenceCount() << vtkstd::endl;    
    Grid[i] = Column;
    vtkstd::cout << "After Grid[" << i << "]=Column[0]: " << Column[0]->GetReferenceCount() << vtkstd::endl;
    vtkstd::cout << "After Grid[" << i << "]=Column[1]: " << Column[1]->GetReferenceCount() << vtkstd::endl;
  }
  std::cout << "-------------- End Of for(i) ---------------" << std::endl;      
  vtkstd::cout << "After leave for(i) scope Grid[0][0]: " << Grid[0][0]->GetReferenceCount() << vtkstd::endl;
  vtkstd::cout << "After leave for(i) scope Grid[0][1]: " << Grid[0][1]->GetReferenceCount() << vtkstd::endl;
  vtkstd::cout << "After leave for(i) scope Grid[1][0]: " << Grid[1][0]->GetReferenceCount() << vtkstd::endl;
  vtkstd::cout << "After leave for(i) scope Grid[1][1]: " << Grid[1][1]->GetReferenceCount() << vtkstd::endl;
  std::cout << "-------------- End Of Program ---------------" << std::endl;  
  return 0;
}
