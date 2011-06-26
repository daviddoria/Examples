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
      LidarPoint->SetRay(Ray);
      
      vtkstd::cout << "Ref Before: " << LidarPoint->GetReferenceCount() << vtkstd::endl;
      Column[j] = LidarPoint;
      vtkstd::cout << "Ref After: " << LidarPoint->GetReferenceCount() << vtkstd::endl;
      
    }
    
    Grid[i] = Column;
    vtkstd::cout << "Ref after column added to grid: " << Grid[0][0]->GetReferenceCount() << vtkstd::endl;
  }
  
  return 0;
}