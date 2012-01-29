#include "project_config.h"

#include "vtkActor.h" 
#include "vtkConeSource.h" 
#include "vtkInteractorStyleTrackballCamera.h" 
#include "vtkPolyDataMapper.h" 
#include "vtkRenderWindow.h" 
#include "vtkRenderWindowInteractor.h" 
#include "vtkRenderer.h" 


int main(int argc, char* argv[])
{
	//Create a cone source object

	vtkConeSource *cone = vtkConeSource::New();
	cone->SetHeight(3.0);
	cone->SetRadius(1.0);
	cone->SetResolution(10);

	//Create a cone mapper object

	vtkPolyDataMapper *coneMapper = vtkPolyDataMapper::New();
	coneMapper->SetInputConnection(cone->GetOutputPort());

	//Associate the cone mapper to an actor object

	vtkActor *coneActor = vtkActor::New();
	coneActor->SetMapper(coneMapper);

	//Add that actor to the renderer

	vtkRenderer *ren1 = vtkRenderer::New();
	ren1->AddActor(coneActor);
	ren1->SetBackground(0.1, 0.2, 0.4);


	//Create a render window

	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	renWin->SetSize(300, 300);

	//Create an interactor and associate it to the render window

	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	//Define the interacting style

	vtkInteractorStyleTrackballCamera *style = vtkInteractorStyleTrackballCamera::New();
	iren->SetInteractorStyle(style);

	//Start to interact

	iren->Initialize();
	iren->Start();


	return 0;
}
