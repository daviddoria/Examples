#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkRectangularButtonSource.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkObjectFactory.h>
#include <vtkProperty.h>


class MouseInteractorStyle : public vtkInteractorStyleTrackballActor
{
  public:
    static MouseInteractorStyle* New();
    vtkTypeRevisionMacro(MouseInteractorStyle,vtkInteractorStyleTrackballActor);
   
    virtual void OnLeftButtonDown() 
    {
      cout << "Pressed left mouse button." << endl;
      
      if(this->InteractionProp)
        {
        vtkActor::SafeDownCast(this->InteractionProp)->GetProperty()->SetColor(1,0,0);
        this->RenderWindow->Render();
        }
      
      // forward events
      vtkInteractorStyleTrackballActor::OnLeftButtonDown();
    }
    
    void SetRenderWindow(const vtkSmartPointer<vtkRenderWindow> &renderWindow) {this->RenderWindow = renderWindow;}

  private:
    vtkSmartPointer<vtkRenderWindow> RenderWindow;
};
 
vtkCxxRevisionMacro(MouseInteractorStyle, "$Revision: 1.1 $");
vtkStandardNewMacro(MouseInteractorStyle);

int main (int argc, char* argv[])
{

  vtkSmartPointer<vtkRectangularButtonSource> button = 
      vtkSmartPointer<vtkRectangularButtonSource>::New();
  
  //create a mapper
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInput(button->GetOutput());

	// create an actor
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
  
  vtkSmartPointer<MouseInteractorStyle> style = 
      vtkSmartPointer<MouseInteractorStyle>::New();
  renderWindowInteractor->SetInteractorStyle( style );
  style->SetRenderWindow(renderWindow);
  
  // add the actors to the scene
  renderer->AddActor(actor);
  renderer->SetBackground(1,1,1); // Background color white

  //setup the text and add it to the window
  vtkSmartPointer<vtkTextActor> textActor = 
      vtkSmartPointer<vtkTextActor>::New();
  textActor->GetTextProperty()->SetFontSize ( 24 );
  textActor->SetPosition2 ( 10, 40 );
  renderer->AddActor2D ( textActor );
  textActor->SetInput ( "Hello world" );
  textActor->GetTextProperty()->SetColor ( 1.0,0.0,0.0 );

  // render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();

  style->SetRenderWindow(NULL);
   
  return EXIT_SUCCESS;
}
