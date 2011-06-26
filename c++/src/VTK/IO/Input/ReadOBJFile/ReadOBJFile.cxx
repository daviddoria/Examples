#include <vtkSmartPointer.h>
#include <vtkOBJReader.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCell.h>
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>

int main(int argc, char *argv[])
{
  //parse command line arguments
  if(argc != 2)
    {
    vtkstd::cout << "Required arguments: Filename" << vtkstd::endl;
    return EXIT_FAILURE;
    }
  
  vtkstd::string filename = argv[1];
  vtkSmartPointer<vtkOBJReader> reader = 
      vtkSmartPointer<vtkOBJReader>::New();
  reader->SetFileName(filename.c_str());
  reader->Update();
    
  vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
  vtkIdType NumPoints = polydata->GetNumberOfPoints();
  
  double point[3];
  for(int i = 0; i < NumPoints; i++)
    {
    polydata->GetPoint(i, point);
    vtkstd::cout << "Point " << i << ": " << point[0] << " " << point[1] << " " << point[2] << vtkstd::endl;
    }
  
  //create triangle vertex lists
  vtkIdType NumPolys = polydata->GetNumberOfPolys();
  if(NumPolys > 0)
    {
    vtkSmartPointer<vtkCellArray> TriangleCells = polydata->GetPolys();
    vtkIdType npts;
    vtkIdType *pts;

    while(TriangleCells->GetNextCell(npts, pts))
      {
      vtkstd::cout << "Indices of triangle points: " << pts[0] << " " << pts[1] << " " << pts[2] << vtkstd::endl;
      }	
    }
  
  return EXIT_SUCCESS;
}
