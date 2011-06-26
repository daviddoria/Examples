#include <vtkSmartPointer.h>
#include <vtkPointSource.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkAppendFilter.h>
#include <vtkUnstructuredGrid.h>

int main(int argc, char *argv[])
{
  //polydata
  vtkSmartPointer<vtkPointSource> pointSource = 
      vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetNumberOfPoints(5);
  pointSource->Update();
  
  vtkSmartPointer<vtkPolyData> polydata = pointSource->GetOutput();
  
  cout << "There are " << polydata->GetNumberOfPoints() << " points in the polydata." << endl;
  
  //unstructured grid
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0,0,0);
  points->InsertNextPoint(0,0,1);
  
  vtkSmartPointer<vtkUnstructuredGrid> ug = 
      vtkSmartPointer<vtkUnstructuredGrid>::New();
  ug->SetPoints(points);
  
  cout << "There are " << ug->GetNumberOfPoints() << " points in the unstructured grid." << endl;
    
  //combine
  vtkSmartPointer<vtkAppendFilter> appendFilter = 
      vtkSmartPointer<vtkAppendFilter>::New();
  appendFilter->AddInput(polydata);
  appendFilter->AddInput(ug);
  appendFilter->Update();
  
  vtkSmartPointer<vtkUnstructuredGrid> combined = appendFilter->GetOutput();
  cout << "There are " << combined->GetNumberOfPoints() << " points combined." << endl;
    
  return EXIT_SUCCESS;
}
