#include "vtkSeedWidget.h"
#include "vtkSeedRepresentation.h"
#include "vtkSphereSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkCommand.h"
#include "vtkInteractorEventRecorder.h"
#include "vtkRegressionTestImage.h"
#include "vtkDebugLeaks.h"
#include "vtkCoordinate.h"
#include "vtkMath.h"
#include "vtkHandleWidget.h"
#include "vtkPointHandleRepresentation2D.h"
#include "vtkAxisActor2D.h"
#include "vtkProperty2D.h"

// This callback is responsible for setting the seed label.
class vtkSeedCallback : public vtkCommand
{
  public:
    static vtkSeedCallback *New() 
    { return new vtkSeedCallback; }
    virtual void Execute(vtkObject*, unsigned long event, void *calldata)
    {
      if (event == vtkCommand::PlacePointEvent)
      {
        cout << "Point placed, total of: " 
            << this->SeedRepresentation->GetNumberOfSeeds() << endl;
      }
      if (event == vtkCommand::InteractionEvent)
      {
        if (calldata)
        {
          cout << "Interacting with seed : " 
              << *(static_cast< int * >(calldata)) << endl;
        }
      }
    }
    vtkSeedCallback() : SeedRepresentation(0) {}
    vtkSeedRepresentation *SeedRepresentation;
};


// The actual test function
int main( int argc, char *argv[] )
{
  // Create the RenderWindow, Renderer and both Actors
  //
  vtkRenderer *ren1 = vtkRenderer::New();
  vtkRenderWindow *renWin = vtkRenderWindow::New();
  renWin->AddRenderer(ren1);

  vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
  iren->SetRenderWindow(renWin);

  // Create a test pipeline
  //
  vtkSphereSource *ss = vtkSphereSource::New();
  vtkPolyDataMapper *mapper = vtkPolyDataMapper::New();
  mapper->SetInput(ss->GetOutput());
  vtkActor *actor = vtkActor::New();
  actor->SetMapper(mapper);

  // Create the widget and its representation
  vtkPointHandleRepresentation2D *handle = vtkPointHandleRepresentation2D::New();
  handle->GetProperty()->SetColor(1,0,0);
  vtkSeedRepresentation *rep = vtkSeedRepresentation::New();
  rep->SetHandleRepresentation(handle);

  vtkSeedWidget *widget = vtkSeedWidget::New();
  widget->SetInteractor(iren);
  widget->SetRepresentation(rep);

  vtkSeedCallback *scbk = vtkSeedCallback::New();
  scbk->SeedRepresentation = rep;
  widget->AddObserver(vtkCommand::PlacePointEvent,scbk);
  widget->AddObserver(vtkCommand::InteractionEvent,scbk);
  widget->On();
  // Add the actors to the renderer, set the background and size
  //
  ren1->AddActor(actor);
  ren1->SetBackground(0.1, 0.2, 0.4);
  renWin->SetSize(300, 300);

  // render the image
  
  iren->Initialize();
  renWin->Render();
  

  // Remove the observers so we can go interactive. Without this the "-I"
  // testing option fails.

  iren->Start();
  
  ss->Delete();
  mapper->Delete();
  actor->Delete();
  handle->Delete();
  rep->Delete();
  widget->RemoveObserver(scbk);
  scbk->Delete();
  widget->Off();
  widget->Delete();
  iren->Delete();
  renWin->Delete();
  ren1->Delete();
  
  return 0;
}