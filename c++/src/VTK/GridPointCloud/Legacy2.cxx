#include <vtkSmartPointer.h>
#include <vtkPointSource.h>
#include <vtkPoints.h>
#include <vtkImageData.h>
#include <vtkCellLocator.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  
  //cluster 1
  points->InsertNextPoint(0.1, 0.1, 0.1);
  points->InsertNextPoint(0.2, 0.2, 0.2);
  points->InsertNextPoint(0.3, 0.3, 0.3);
  
  //cluster 2
  points->InsertNextPoint(1.1, 1.1, 1.1);
  points->InsertNextPoint(1.2, 1.2, 1.2);
  points->InsertNextPoint(1.3, 1.3, 1.3);
  
  vtkSmartPointer<vtkPolyData> polydata = 
      vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
  
  double bounds[6];
  polydata->GetBounds(bounds);
  
  vtkSmartPointer<vtkImageData> grid =
      vtkSmartPointer<vtkImageData>::New();
  grid->SetOrigin(bounds[0], bounds[2], bounds[4]);
  
  unsigned int numVoxels = 2; //the number of voxels in each dimension
  grid->SetSpacing((bounds[1]-bounds[0])/static_cast<double>(numVoxels), (bounds[3]-bounds[2])/numVoxels, (bounds[5]-bounds[4])/static_cast<double>(numVoxels));
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
  
  //determine which cell each point is in
  cout << "Which cell does each point belong to?" << endl;
  for(vtkIdType i = 0; i < polydata->GetNumberOfPoints(); i++)
    {
    double p[3];
    polydata->GetPoint(i,p);
    int cellId = cellLocator->FindCell(p);
    cout << "Point " << i << " is in cell " << cellId << endl;
    }
  
  cout << endl;
  
  //determine which points are in a particular cell
  cout << "Which points are in each cell?" << endl;
  for(vtkIdType i = 0; i < grid->GetNumberOfCells(); i++)
    {
    
    }
    
  return EXIT_SUCCESS;
}
