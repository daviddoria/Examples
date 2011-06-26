#include <vtkSmartPointer.h>
#include <vtkWidgetEvent.h>
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyDataMapper.h>
#include <vtkHoverWidget.h>
#include <vtkPointHandleRepresentation2D.h>
#include <vtkProperty2D.h>

class vtkHoverCallback : public vtkCommand
{
  public:
    static vtkHoverCallback *New()
    { 
      return new vtkHoverCallback; 
    }
    
    vtkHoverCallback() {}
    
    virtual void Execute(vtkObject*, unsigned long event, void *calldata)
    {
      /*
      if (event == vtkCommand::TimerEvent)
        {
        cout << "TimerEvent." << endl;
        }
      if (event == vtkCommand::Move)
        {
        cout << "Move" << endl;
        }
        */
      
      if (event == vtkWidgetEvent::Move)
        {
        cout << "Move." << endl;
        }
      if (event == vtkWidgetEvent::TimedOut)
        {
        cout << "TimedOut." << endl;
        }
/*
              if (event == vtkWidgetEvent::SelectAction)
        {
        cout << "SelectAction." << endl;
        }
        */
    }
    
};

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  
  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(sphereSource->GetOutputPort());
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  
  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  renderer->AddActor(actor);
  
  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  //create the widget
  vtkSmartPointer<vtkHoverWidget> hoverWidget = 
      vtkSmartPointer<vtkHoverWidget>::New();
  hoverWidget->SetInteractor(renderWindowInteractor);
  hoverWidget->SetTimerDuration(1000);
  
  vtkSmartPointer<vtkHoverCallback> hoverCallback = 
      vtkSmartPointer<vtkHoverCallback>::New();
  hoverWidget->AddObserver(vtkCommand::TimerEvent,hoverCallback);
    
  renderWindow->Render();
  
  renderWindowInteractor->Initialize();
  renderWindow->Render();
  hoverWidget->On();
  
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
