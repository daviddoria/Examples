#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkCellArray.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0,0,0);
  points->InsertNextPoint(1,1,1);
  points->InsertNextPoint(2,2,2);
  
  vtkSmartPointer<vtkPolyData> polydata =
    vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
  
  std::cout << "Before filter:" << std::endl;
  std::cout << "There are " << polydata->GetNumberOfPoints() << " points." << std::endl;
  std::cout << "There are " << polydata->GetNumberOfCells() << " cells." << std::endl;

  vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter =
    vtkSmartPointer<vtkVertexGlyphFilter>::New();
  vertexGlyphFilter->AddInput(polydata);
  vertexGlyphFilter->Update();
  
  vtkPolyData* vertexPolyData = vertexGlyphFilter->GetOutput();
    
  std::cout << "After filter:" << std::endl;
  std::cout << "There are " << vertexPolyData->GetNumberOfPoints() << " points." << std::endl;
  std::cout << "There are " << vertexPolyData->GetNumberOfCells() << " cells." << std::endl;

  return EXIT_SUCCESS;
}
