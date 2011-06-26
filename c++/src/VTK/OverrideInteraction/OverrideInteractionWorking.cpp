#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSphereSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkMath.h"
#include "vtkProperty.h"

// Define interaction style
class MyInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static MyInteractorStyle* New()
    { return new MyInteractorStyle();}

    void SetActor(vtkActor* actor) { mActor = actor ; }
    void SetSource(vtkSphereSource* src) {mSrc = src;}

    virtual void OnChar() 
    {
      vtkRenderWindowInteractor *rwi = this->Interactor;		
      char ch = rwi->GetKeyCode() ;

      switch (ch) 
      {
        case '+':
        case '-':
		
          int resPhi, resTheta ;
          resPhi = mSrc->GetPhiResolution();
          resTheta = mSrc->GetThetaResolution();

          if (ch=='+') {resPhi++; resTheta++; }
          else if (ch=='-' && resPhi>0) { resPhi--; }
          else if (ch=='-' && resTheta>0) {resTheta--; }
			
          mSrc->SetPhiResolution(resPhi);
          mSrc->SetThetaResolution(resTheta);

          rwi->Render(); // render - update the screen
          break ;

        case 'c':

          double r,g,b ;

          r = vtkMath::Random(0,1);
          g = vtkMath::Random(0,1);
          b = vtkMath::Random(0,1);

          mActor->GetProperty()->SetColor(r,g,b);

          rwi->Render(); // render - update the screen
          break ;

      }
	
		// forward events
      vtkInteractorStyleTrackballCamera::OnChar();
    }
  private:
    vtkActor* mActor ;
    vtkSphereSource* mSrc ;
};

int main(int argc, char* agrv[])
{
	// Create a renderer
  vtkRenderer* ren = vtkRenderer::New();
  ren->SetBackground(0.0,0.0,0.0);

	// Create a sphere and add to the renderer
  vtkSphereSource* src = vtkSphereSource::New();
  vtkPolyDataMapper* mapper = vtkPolyDataMapper::New();
  mapper->SetInputConnection(src->GetOutputPort());
  vtkActor* actor = vtkActor::New();
  actor->SetMapper(mapper);
  ren->AddActor(actor);
	
	// Create a render window
  vtkRenderWindow* renWin = vtkRenderWindow::New();
  renWin->AddRenderer( ren );
  renWin->SetSize( 800, 600);

	// Create an interactor
  vtkRenderWindowInteractor* iren = vtkRenderWindowInteractor::New();
  renWin->SetInteractor( iren );

	// Create my interactor style
  MyInteractorStyle* style = MyInteractorStyle::New();
  style->SetActor(actor);
  style->SetSource(src);
  iren->SetInteractorStyle( style );

	// Initialize and enter interactive mode
  iren->Initialize();
  iren->Start();

  return 0 ;
}