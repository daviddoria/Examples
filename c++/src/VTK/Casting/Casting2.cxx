#include <vtkSmartPointer.h>
#include <vtkTriangle.h>
#include <vtkQuad.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>

int main ( int argc, char *argv[] )
{
  //create points
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();

  unsigned int NumberOfPoints = 3;
  points->InsertNextPoint ( 0.0, 0.0, 0.0 );
  points->InsertNextPoint ( 1.0, 0.0, 0.0 );
  points->InsertNextPoint ( 0.0, 1.0, 0.0 );
  points->InsertNextPoint ( 0.0, 0.0, 0.0 );
  points->InsertNextPoint ( 1.0, 0.0, 0.0 );
  points->InsertNextPoint ( 0.0, 1.0, 0.0 );
  points->InsertNextPoint ( 0.0, 1.0, 1.0 );

  vtkSmartPointer<vtkCellArray> cells = 
      vtkSmartPointer<vtkCellArray>::New();
  
  vtkSmartPointer<vtkTriangle> triangle = 
      vtkSmartPointer<vtkTriangle>::New();
  triangle->GetPointIds()->SetId(0, 0);
  triangle->GetPointIds()->SetId(1, 1);
  triangle->GetPointIds()->SetId(2, 2);
  
  vtkSmartPointer<vtkQuad> quad = 
      vtkSmartPointer<vtkQuad>::New();
  quad->GetPointIds()->SetId(0, 0);
  quad->GetPointIds()->SetId(1, 1);
  quad->GetPointIds()->SetId(2, 2);
    
  cells->InsertNextCell(triangle);
  cells->InsertNextCell(quad);
 
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
  polydata->SetPolys(cells);
  
  for(unsigned int i = 0; i < polydata->GetNumberOfCells(); i++)
  {
    vtkCell* cell = polydata->GetCell(i);
 
    vtkTriangle* triangle = dynamic_cast<vtkTriangle*>(cell);
    
    if(triangle)
      {
      cout << "triangle" << endl;
      continue;
      }
      
    vtkQuad* quad = dynamic_cast<vtkQuad*>(cell);
    
    if(quad)
      {
      cout << "quad" << endl;
      continue;
      }
      
  }
  
  return EXIT_SUCCESS;
}

