#include <vtkSmartPointer.h>
#include <vtkMarchingCubes.h>
#include <vtkVoxelModeller.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
 
int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  double bounds[6];
  sphereSource->GetOutput()->GetBounds(bounds);
 
  vtkSmartPointer<vtkVoxelModeller> voxelModeller = 
      vtkSmartPointer<vtkVoxelModeller>::New();
  //voxelModeller->SetSampleDimensions(20,20,20);
  voxelModeller->SetSampleDimensions(50,50,50);
  //voxelModeller->SetSampleDimensions(100,100,100);
  //voxelModeller->SetMaximumDistance(10.0);
  voxelModeller->SetMaximumDistance(0.1);
  voxelModeller->SetModelBounds(bounds);
 
  voxelModeller->SetInputConnection(sphereSource->GetOutputPort());
  voxelModeller->Update();
 
  vtkSmartPointer<vtkMarchingCubes> surface = 
      vtkSmartPointer<vtkMarchingCubes>::New();
 
  surface->SetInputConnection(voxelModeller->GetOutputPort());
  surface->SetNumberOfContours(1);
  surface->ComputeScalarsOn();
  surface->ComputeGradientsOn();
  surface->ComputeNormalsOn();
  //surface->SetValue(0, 0.5);
  surface->SetValue(0, 1.0);
  
 
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  renderer->SetBackground(1, 1, 1); // Background color white
 
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);
 
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(surface->GetOutputPort());
 
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
 
  renderer->AddActor(actor);
 
  renderWindow->Render();
  interactor->Start();
  
  return EXIT_SUCCESS;
}