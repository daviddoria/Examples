#include <vtkVectorText.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>

int main(int argc, char *argv[])
{
  
  //Create a sphere
  vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
  
  vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
  sphereActor->SetMapper(sphereMapper);
  sphereActor->GetProperty()->SetColor(1.0, 0.0, 0.0);

  // Create text
  vtkSmartPointer<vtkVectorText> textSource = vtkSmartPointer<vtkVectorText>::New();
  textSource->SetText("Hello");
  textSource->Update();

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(textSource->GetOutputPort());
  
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(1.0, 0.0, 0.0);
  actor->AddPosition(1,0,0);
  actor->SetScale(5);
  
  //Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  //Add the actor to the scene
  renderer->AddActor(actor);
  renderer->AddActor(sphereActor);
  renderer->SetBackground(1,1,1); // Background color white

  //Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();

  return 0;
}
