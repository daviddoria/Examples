#include <vtkSmartPointer.h>
#include "vtkAffineWidget.h"
#include "vtkAffineRepresentation2D.h"
#include "vtkImageActor.h"
#include "vtkImageData.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkInteractorStyleImage.h"
#include "vtkImageActor.h"
#include "vtkCommand.h"
#include "vtkTransform.h"
#include <vtkCubeSource.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>

// This callback is responsible for adjusting the point position.
// It looks in the region around the point and finds the maximum or
// minimum value.
class vtkAffineCallback : public vtkCommand
{
  public:
    static vtkAffineCallback *New() 
    { return new vtkAffineCallback; }
    virtual void Execute(vtkObject *caller, unsigned long, void*);
    vtkAffineCallback(){Transform = vtkSmartPointer<vtkTransform>::New();}
    
    vtkActor *Actor;
    vtkAffineRepresentation2D *AffineRep;
    vtkSmartPointer<vtkTransform> Transform;
};

// Method re-positions the points using random perturbation
void vtkAffineCallback::Execute(vtkObject*, unsigned long, void*)
{
  this->AffineRep->GetTransform(this->Transform);
  this->Actor->SetUserTransform(this->Transform);
}


int main( int argc, char *argv[] )
{
  vtkSmartPointer<vtkCubeSource> cubeSource = 
      vtkSmartPointer<vtkCubeSource>::New();
  cubeSource->Update();
  
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(cubeSource->GetOutputPort());
  
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
    
  vtkRenderer *ren1 = vtkRenderer::New();
  vtkRenderWindow *renWin = vtkRenderWindow::New();
  renWin->AddRenderer(ren1);

  vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
  iren->SetRenderWindow(renWin);

  vtkInteractorStyleImage *style = vtkInteractorStyleImage::New();
  iren->SetInteractorStyle(style);

  // VTK widgets consist of two parts: the widget part that handles event processing;
  // and the widget representation that defines how the widget appears in the scene 
  // (i.e., matters pertaining to geometry).
  vtkAffineRepresentation2D *rep = vtkAffineRepresentation2D::New();
  rep->SetBoxWidth(100);
  rep->SetCircleWidth(75);
  rep->SetAxesWidth(60);
  rep->DisplayTextOn();
//  rep->PlaceWidget(bounds);

  vtkAffineWidget *widget = vtkAffineWidget::New();
  widget->SetInteractor(iren);
  widget->SetRepresentation(rep);

  vtkAffineCallback *acbk = vtkAffineCallback::New();
  acbk->AffineRep = rep;
  acbk->Actor = actor;
  widget->AddObserver(vtkCommand::InteractionEvent,acbk);
  widget->AddObserver(vtkCommand::EndInteractionEvent,acbk);

  widget->EnabledOn();
  
  // Add the actors to the renderer, set the background and size
  ren1->AddActor(actor);
  ren1->SetBackground(0.1, 0.2, 0.4);
  renWin->SetSize(300, 300);
  
  iren->Initialize();
  renWin->Render();
    
  iren->Start();
  return 0;

}

