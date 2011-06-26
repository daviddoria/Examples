#include "vtkRenderWindow.h"
#include "vtkSmartPointer.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSphereSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkMath.h"
#include "vtkProperty.h"

#include "MyInteractorStyle.h"

int main(int argc, char* agrv[])
{
	// Create a renderer
  vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New();
  ren->SetBackground(0.0,0.0,0.0);

	// Create a sphere and add to the renderer
  vtkSmartPointer<vtkSphereSource> src = vtkSmartPointer<vtkSphereSource>::New();
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(src->GetOutputPort());
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  ren->AddActor(actor);
	
	// Create a render window
  vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
  renWin->AddRenderer( ren );
  renWin->SetSize( 800, 600);

	// Create an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renWin->SetInteractor( iren );

	// Create my interactor style
  vtkSmartPointer<MyInteractorStyle> style = vtkSmartPointer<MyInteractorStyle>::New();
  style->SetActor(actor);
  style->SetSource(src);
  iren->SetInteractorStyle( style );

	// Initialize and enter interactive mode
  iren->Initialize();
  iren->Start();

  return 0 ;
}

