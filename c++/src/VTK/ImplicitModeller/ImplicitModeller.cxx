#include <vtkSmartPointer.h>
#include <vtkImplicitModeller.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkXMLImageDataWriter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkContourFilter.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLImageDataWriter.h>

int main(int, char *[])
{
  // Create a discrete sphere
  vtkSmartPointer<vtkSphereSource> sphereSource = 
    vtkSmartPointer<vtkSphereSource>::New();
  
  vtkSmartPointer<vtkImplicitModeller> implicitModeller = 
    vtkSmartPointer<vtkImplicitModeller>::New();
  implicitModeller->SetSampleDimensions(20,20,20);
  implicitModeller->SetInputConnection(sphereSource->GetOutputPort());
  implicitModeller->Update();
    
  // Create the 0 isosurface
  vtkSmartPointer<vtkContourFilter> contourFilter = 
    vtkSmartPointer<vtkContourFilter>::New();
  contourFilter->SetInputConnection(implicitModeller->GetOutputPort());
  contourFilter->SetNumberOfContours(1);
  contourFilter->SetValue(0, .1);
  contourFilter->Update();
    
  // Visualize
  vtkSmartPointer<vtkRenderer> renderer = 
    vtkSmartPointer<vtkRenderer>::New();
  
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = 
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);
  
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(contourFilter->GetOutputPort());
  
  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  renderer->AddActor(actor);
  renderer->SetBackground(1,1,1); // Background color white
 
  renderWindow->Render();
  interactor->Start();
  
  return EXIT_SUCCESS;
}
