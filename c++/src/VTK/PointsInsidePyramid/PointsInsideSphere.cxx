#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
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
  
  //sphere
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
    
  vtkSmartPointer<vtkPolyDataMapper> surfaceMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  surfaceMapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkSmartPointer<vtkActor> surfaceActor = 
      vtkSmartPointer<vtkActor>::New();
  surfaceActor->SetMapper(surfaceMapper);
  
  cout << "Total points: " << pointSource->GetOutput()->GetNumberOfPoints() << endl;
  //find enclosed points
  vtkSmartPointer<vtkSelectEnclosedPoints> enclosedFilter =
      vtkSmartPointer<vtkSelectEnclosedPoints>::New();
  enclosedFilter->SetInputConnection(pointSource->GetOutputPort());
  enclosedFilter->SetSurface(sphereSource->GetOutput());
  enclosedFilter->Update();

  //extract the enclosed points
  vtkSmartPointer<vtkThresholdPoints> threshold = 
        vtkSmartPointer<vtkThresholdPoints>::New();
  threshold->SetInputConnection(enclosedFilter->GetOutputPort());
  threshold->SetInputArrayToProcess(0,0,0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "SelectedPoints");
  
  threshold->ThresholdByUpper(0.9); //grab all the points that are marked "1" (inside the sphere)
  threshold->Update();
  
  cout << "thresholded points (inside sphere): " << threshold->GetOutput()->GetNumberOfPoints() << endl;
  
  vtkSmartPointer<vtkDataSetMapper> enclosedMapper = 
      vtkSmartPointer<vtkDataSetMapper>::New();
  enclosedMapper->SetInputConnection(threshold->GetOutputPort());
  enclosedMapper->ScalarVisibilityOff();

  vtkSmartPointer<vtkActor> enclosedActor = 
      vtkSmartPointer<vtkActor>::New();
  enclosedActor->SetMapper(enclosedMapper);
  enclosedActor->GetProperty()->SetPointSize(5);
  enclosedActor->GetProperty()->SetColor(0.0, 1.0, 0.0); //(R,G,B) - should be bright green
  
  
  //Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  //renderer->AddActor(pointActor);
  renderer->AddActor(enclosedActor);
  //renderer->AddActor(surfaceActor);
  
  renderer->SetBackground(1,1,1);
  //renderer->SetBackground(1,0,0);
  renderWindow->Render();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}