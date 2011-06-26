#include <vtkSmartPointer.h>
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCamera.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkCommand.h>
#include <vtkBoxWidget.h>
#include <vtkTransform.h>
#include <vtkInteractorStyleTrackballCamera.h>

class vtkMyCallback : public vtkCommand
{
public:
  static vtkMyCallback *New() 
    { return new vtkMyCallback; }
  virtual void Execute(vtkObject *caller, unsigned long, void*)
    {
      vtkSmartPointer<vtkTransform> t = 
	vtkSmartPointer<vtkTransform>::New();
      vtkBoxWidget *widget = reinterpret_cast<vtkBoxWidget*>(caller);
      widget->GetTransform(t);
      widget->GetProp3D()->SetUserTransform(t);
    }
};

int main(int, char*[])
{
  
  vtkSmartPointer<vtkConeSource> cone = 
    vtkSmartPointer<vtkConeSource>::New();
  cone->SetHeight( 3.0 );
  cone->SetRadius( 1.0 );
  cone->SetResolution( 10 );
  cone->Update();
  
  vtkSmartPointer<vtkPolyDataMapper> coneMapper = 
    vtkSmartPointer<vtkPolyDataMapper>::New();
  coneMapper->SetInputConnection( cone->GetOutputPort() );

  vtkSmartPointer<vtkActor> coneActor = 
    vtkSmartPointer<vtkActor>::New();
  coneActor->SetMapper( coneMapper );

  vtkSmartPointer<vtkRenderer> ren1= 
    vtkSmartPointer<vtkRenderer>::New();
  ren1->AddActor( coneActor );
  ren1->SetBackground( 0.1, 0.2, 0.4 );

  vtkSmartPointer<vtkRenderWindow> renWin = 
    vtkSmartPointer<vtkRenderWindow>::New();
  renWin->AddRenderer( ren1 );
  renWin->SetSize( 300, 300 );

  vtkSmartPointer<vtkRenderWindowInteractor> iren = 
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  iren->SetRenderWindow(renWin);

  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = 
    vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
  iren->SetInteractorStyle(style);

  //
  // Here we use a vtkBoxWidget to transform the underlying coneActor (by
  // manipulating its transformation matrix).
  
  // The place factor 
  // controls the initial size of the widget with respect to the bounding box
  // of the input to the widget.
  vtkSmartPointer<vtkBoxWidget> boxWidget = 
    vtkSmartPointer<vtkBoxWidget>::New();
  boxWidget->SetInteractor(iren);
  boxWidget->SetPlaceFactor(1.25);

  boxWidget->SetProp3D(coneActor);
  boxWidget->PlaceWidget();
  vtkMyCallback *callback = vtkMyCallback::New();
  boxWidget->AddObserver(vtkCommand::InteractionEvent, callback);

  boxWidget->On();

  iren->Initialize();
  iren->Start();
  
  return EXIT_SUCCESS;
}

