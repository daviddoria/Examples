#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPyramid.h>
#include <vtkUnstructuredGrid.h>
#include <vtkDataSetMapper.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPointSource.h>
#include <vtkSelectEnclosedPoints.h>
#include <vtkProperty.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkThreshold.h>
#include <vtkThresholdPoints.h>
#include <vtkPointData.h>
#include <vtkVertexGlyphFilter.h>

int main(int argc, char *argv[])
{
  //point cloud
  vtkSmartPointer<vtkPointSource> pointSource = 
      vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetNumberOfPoints(500);
  pointSource->SetRadius(1);
  pointSource->Update();
  
  vtkSmartPointer<vtkDataSetMapper> pointMapper = 
      vtkSmartPointer<vtkDataSetMapper>::New();
  pointMapper->SetInputConnection(pointSource->GetOutputPort());

  vtkSmartPointer<vtkActor> pointActor = 
      vtkSmartPointer<vtkActor>::New();
  pointActor->SetMapper(pointMapper);
  
  //pyramid
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  
  float p0[3] = {1.0, 1.0, 1.0};
  float p1[3] = {-1.0, 1.0, 1.0};
  float p2[3] = {-1.0, -1.0, 1.0};
  float p3[3] = {1.0, -1.0, 1.0};
  float p4[3] = {0.0, 0.0, 0.0};
    
  points->InsertNextPoint(p0);
  points->InsertNextPoint(p1);
  points->InsertNextPoint(p2);
  points->InsertNextPoint(p3);
  points->InsertNextPoint(p4);
  
  vtkSmartPointer<vtkPyramid> pyramid = 
      vtkSmartPointer<vtkPyramid>::New();
  pyramid->GetPointIds()->SetId(0,0);
  pyramid->GetPointIds()->SetId(1,1);
  pyramid->GetPointIds()->SetId(2,2);
  pyramid->GetPointIds()->SetId(3,3);
  pyramid->GetPointIds()->SetId(4,4);
  
  vtkSmartPointer<vtkCellArray> cells = 
      vtkSmartPointer<vtkCellArray>::New();
  cells->InsertNextCell (pyramid);
  
  vtkSmartPointer<vtkUnstructuredGrid> ug = 
      vtkSmartPointer<vtkUnstructuredGrid>::New();
  ug->SetPoints(points);
  ug->InsertNextCell(pyramid->GetCellType(),pyramid->GetPointIds());
    
  //get pyramid surface
  vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter = 
      vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
  surfaceFilter->SetInput(ug);
  surfaceFilter->Update();
  
  vtkSmartPointer<vtkPolyDataMapper> surfaceMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  surfaceMapper->SetInputConnection(surfaceFilter->GetOutputPort());

  vtkSmartPointer<vtkActor> surfaceActor = 
      vtkSmartPointer<vtkActor>::New();
  surfaceActor->SetMapper(surfaceMapper);
  
  cout << "Total points: " << pointSource->GetOutput()->GetNumberOfPoints() << endl;
  //find enclosed points
  vtkSmartPointer<vtkSelectEnclosedPoints> enclosedFilter =
      vtkSmartPointer<vtkSelectEnclosedPoints>::New();
  enclosedFilter->SetInputConnection(pointSource->GetOutputPort());
  enclosedFilter->SetSurfaceConnection(surfaceFilter->GetOutputPort());
  enclosedFilter->Update();

  //extract the enclosed points
  vtkSmartPointer<vtkThresholdPoints> threshold = 
        vtkSmartPointer<vtkThresholdPoints>::New();
  threshold->SetInputConnection(enclosedFilter->GetOutputPort());
  threshold->SetInputArrayToProcess(0,0,0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "SelectedPoints");
  
  threshold->ThresholdByUpper(0.9); //grab all the points that are marked "1"
  threshold->Update();
  
  cout << "thresholded points: " << threshold->GetOutput()->GetNumberOfPoints() << endl;
    
  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = 
      vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputConnection(threshold->GetOutputPort());
  glyphFilter->Update();
  
  vtkSmartPointer<vtkDataSetMapper> enclosedMapper = 
      vtkSmartPointer<vtkDataSetMapper>::New();
  enclosedMapper->SetInputConnection(glyphFilter->GetOutputPort());

  vtkSmartPointer<vtkActor> enclosedActor = 
      vtkSmartPointer<vtkActor>::New();
  enclosedActor->SetMapper(enclosedMapper);
  enclosedActor->GetProperty()->SetPointSize(5);
  enclosedActor->GetProperty()->SetColor(0.0, 1.0, 0.0); //(R,G,B)
  //enclosedActor->GetProperty()->SetColor(255.0, 0.0, 0.0);
  
  //Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(pointActor);
  renderer->AddActor(enclosedActor);
  renderer->AddActor(surfaceActor);
  
  //renderer->SetBackground(1,1,1);
  renderer->SetBackground(1,0,0);
  renderWindow->Render();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
