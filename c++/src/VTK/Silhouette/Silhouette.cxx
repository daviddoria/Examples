#include <vtkSmartPointer.h>
#include <vtkProperty.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyDataSilhouette.h>
#include <vtkSphereSource.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  //create mapper and actor for original model
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(sphereSource->GetOutputPort());
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  
  //create renderer and renderWindow
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  
  //renderer->AddActor(actor); //view the original model
  
  //Compute the silhouette
  vtkSmartPointer<vtkPolyDataSilhouette> silhouette = 
      vtkSmartPointer<vtkPolyDataSilhouette>::New();
  silhouette->SetInputConnection(sphereSource->GetOutputPort());
  silhouette->SetCamera(renderer->GetActiveCamera());
  silhouette->Update();
  
    //you MUST NOT call renderWindow->Render() before iren->SetRenderWindow(renderWindow);
  vtkSmartPointer<vtkRenderWindowInteractor> iren = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  iren->SetRenderWindow(renderWindow);
  
  //create mapper and actor for silouette
  vtkSmartPointer<vtkPolyDataMapper> mapper2 = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper2->SetInputConnection(silhouette->GetOutputPort());
  vtkSmartPointer<vtkActor> actor2 = vtkSmartPointer<vtkActor>::New();
  actor2->SetMapper(mapper2);
  //actor2->GetProperty()->SetColor(0.5, 0.5, 1);
  renderer->AddActor(actor2);
  renderer->ResetCamera();
  
  //render and interact
  renderWindow->Render();
  iren->Start();

  return EXIT_SUCCESS;
}
