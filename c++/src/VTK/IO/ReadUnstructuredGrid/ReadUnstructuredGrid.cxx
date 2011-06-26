#include <vtkSmartPointer.h>
#include <vtkXMLUnstructuredGridReader.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCell.h>
#include <vtkIdList.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>

int main(int argc, char *argv[])
{
  //parse command line arguments
  if(argc != 2)
    {
    cout << "Required arguments: Filename" << endl;
    return EXIT_FAILURE;
    }
  
  vtkstd::string filename = argv[1];
    
  //read all the data from the file
  vtkSmartPointer<vtkXMLUnstructuredGridReader> reader = 
      vtkSmartPointer<vtkXMLUnstructuredGridReader>::New();
  reader->SetFileName(filename.c_str());
  reader->Update();
  vtkUnstructuredGrid* unstructuredGrid = reader->GetOutput();

  //get points
  vtkIdType numPoints = unstructuredGrid->GetNumberOfPoints();
  cout << "There are " << numPoints << " points." << endl;
  
  if(!(numPoints > 0) )
    {
    return EXIT_FAILURE;
    }
  
  double point[3];
  for(unsigned int i = 0; i < numPoints; i++)
  {
    unstructuredGrid->GetPoint(i, point);
    cout << "Point " << i << ": " << point[0] << " " << point[1] << " " << point[2] << endl;
  }
  
  //get triangles
  //if( (ug->GetCellType(0) == VTK_TRIANGLE) && (NumCells > 0) )//vtkCellType.h
  vtkIdType numCells = unstructuredGrid->GetNumberOfCells();
  vtkstd::cout << "There are " << numCells << " cells." << vtkstd::endl;
  for(vtkIdType tri = 0; tri < numCells; tri++)
    {
    vtkSmartPointer<vtkCell> cell = unstructuredGrid->GetCell(tri);
    vtkSmartPointer<vtkIdList> pts = cell->GetPointIds();
    std::vector<int> list(3);
    list[0] = pts->GetId(0);
    list[1] = pts->GetId(1);
    list[2] = pts->GetId(2);
    }
  
  //get colors
  vtkSmartPointer<vtkUnsignedCharArray> colorsData = vtkUnsignedCharArray::SafeDownCast(
      unstructuredGrid->GetPointData()->GetArray("Colors"));
  
  if(colorsData)
    { 
    unsigned char color[3];
      
    for(unsigned int i = 0; i < static_cast<unsigned int> (numPoints); i++)
      {
      colorsData->GetTupleValue(i, color);
      cout << "Color " << i << ": " << color[0] << " " << color[1] << " " << color[2] << endl;
      }
    }
  
  return EXIT_SUCCESS;
}

