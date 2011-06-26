#include <vtkSmartPointer.h>
#include <vtkPointSource.h>
#include <vtkPoints.h>
#include <vtkImageData.h>
#include <vtkCellLocator.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPointSource> pointSource = 
      vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetNumberOfPoints(10);
  pointSource->Update();
  
  double bounds[6];
  pointSource->GetOutput()->GetBounds(bounds);
  
  vtkSmartPointer<vtkImageData> grid =
      vtkSmartPointer<vtkImageData>::New();
  grid->SetOrigin(bounds[0], bounds[2], bounds[4]);
  double voxelSize = .1;
  double numVoxels = 10.0; //the number of voxels in each dimension
  grid->SetSpacing((bounds[1]-bounds[0])/numVoxels, (bounds[3]-bounds[2])/numVoxels, (bounds[5]-bounds[4])/numVoxels);
  int extent[6];
  extent[0] = 0;
  extent[1] = numVoxels;
  extent[2] = 0;
  extent[3] = numVoxels;
  extent[4] = 0;
  extent[5] = numVoxels;
  grid->SetExtent(extent);
  grid->Update();
  
  vtkSmartPointer<vtkCellLocator> cellLocator = 
      vtkSmartPointer<vtkCellLocator>::New();
  cellLocator->SetDataSet(grid);
  cellLocator->BuildLocator();
  
  for(vtkIdType i = 0; i < pointSource->GetOutput()->GetNumberOfPoints(); i++)
    {
    double p[3];
    pointSource->GetOutput()->GetPoint(i,p);
    int cellId = cellLocator->FindCell(p);
    cout << "Point " << i << " is in cell " << cellId << endl;
    }
  
  return EXIT_SUCCESS;
}
