#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkCaptionWidget.h>
#include <vtkCaptionRepresentation.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>

int main (int argc, char *argv[])
{

  //sphere
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
    
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInput(sphereSource->GetOutput());

  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  
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
  
  // Create the widget and its representation
  vtkSmartPointer<vtkCaptionRepresentation> captionRepresentation = 
      vtkSmartPointer<vtkCaptionRepresentation>::New();
  captionRepresentation->GetCaptionActor2D()->SetCaption("Test caption");
  captionRepresentation->GetCaptionActor2D()->GetTextActor()->GetTextProperty()->SetFontSize(100);
  
  double pos[3] = {.5,0,0};
  captionRepresentation->SetAnchorPosition(pos);
  
  vtkSmartPointer<vtkCaptionWidget> captionWidget = 
      vtkSmartPointer<vtkCaptionWidget>::New();
  captionWidget->SetInteractor(renderWindowInteractor);
  captionWidget->SetRepresentation(captionRepresentation);
  
  // add the actors to the scene
  renderer->AddActor(actor);
  renderer->SetBackground(1,0,0);

  renderWindow->Render();
  captionWidget->On();
  
  // begin mouse interaction
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
