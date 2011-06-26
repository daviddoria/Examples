#include <vtkSmartPointer.h>
#include <vtkCell.h>
#include <vtkIdList.h>
#include <vtkPolyDataMapper.h>
#include <vtkObjectFactory.h>
#include <vtkPointSource.h>
#include <vtkPolyData.h>
#include <vtkPointLocator.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPointSource> pointSource = 
      vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetNumberOfPoints(400);
  pointSource->Update();
  
  //Create the tree
  vtkSmartPointer<vtkPointLocator> pointLocator = 
      vtkSmartPointer<vtkPointLocator>::New();
  pointLocator->SetDataSet(pointSource->GetOutput());
  pointLocator->AutomaticOn();
  pointLocator->SetNumberOfPointsPerBucket(2);
  pointLocator->BuildLocator();
  
  vtkSmartPointer<vtkPolyData> polydata = 
      vtkSmartPointer<vtkPolyData>::New();
  
  //pointLocator->GenerateRepresentation(1, polydata);
  pointLocator->GenerateRepresentation(pointLocator->GetLevel(), polydata);
  
  cout << "There are " << polydata->GetNumberOfPoints() << " points." << endl;
  cout << "There are " << polydata->GetNumberOfCells() << " voxels." << endl;
  cout << polydata->GetCell(0)->GetClassName() << endl;
  
  //vtkVoxel* voxel = vtkVoxel::SafeDownCast(imageData->GetCell(i));
  return EXIT_SUCCESS;
}
