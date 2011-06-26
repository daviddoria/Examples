#include <vtkPolyDataMapper.h>
#include <vtkQuadricLODActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>

int main (int argc, char *argv[])
{
  //high res sphere
  vtkSmartPointer<vtkSphereSource> highResSphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  highResSphereSource->SetThetaResolution(200);
  highResSphereSource->SetPhiResolution(200);
  highResSphereSource->Update();
  
  vtkSmartPointer<vtkPolyDataMapper> highResMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  highResMapper->SetInputConnection(highResSphereSource->GetOutputPort());
  
  vtkSmartPointer<vtkQuadricLODActor> actor = 
      vtkSmartPointer<vtkQuadricLODActor>::New();
  actor->SetMapper(highResMapper);
  
  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderWindowInteractor->SetDesiredUpdateRate(1e20);
      
  renderer->AddActor(actor);
  renderWindow->Render();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
