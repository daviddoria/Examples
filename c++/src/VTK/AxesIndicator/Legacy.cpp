#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkPolyData.h"
#include "vtkSphereSource.h"
#include "vtkAxesActor.h"
#include "vtkOrientationMarkerWidget.h"

void SphereDemo();

int main ()
{
	SphereDemo();
	return 0;
}

void SphereDemo()
{
	vtkSphereSource* sphere = vtkSphereSource::New();
	sphere->SetCenter(0.0, 0.0, 0.0);
	sphere->SetRadius(5.0);
	vtkPolyData* polydata = sphere->GetOutput();

	vtkAxesActor* AxesActor = vtkAxesActor::New();
	AxesActor->SetShaftTypeToCylinder();
	AxesActor->SetXAxisLabelText( "X" );
	AxesActor->SetYAxisLabelText( "Y" );
	AxesActor->SetZAxisLabelText( "Z" );
	AxesActor->SetTotalLength( 1.5, 1.5, 1.5 );
	
	vtkOrientationMarkerWidget* AxesWidget = vtkOrientationMarkerWidget::New();


	// map the contours to graphical primitives
	vtkPolyDataMapper *contMapper = vtkPolyDataMapper::New();
	contMapper->SetInput(polydata);

  	// create an actor for the contours
	vtkActor *contActor = vtkActor::New();
	contActor->SetMapper(contMapper);

  	// a renderer and render window
	vtkRenderer *Renderer = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(Renderer);


	// add the actors to the scene
	Renderer->AddActor(contActor);
	Renderer->SetBackground(1,1,1); // Background color white
	//Renderer->AddViewProp(AxesActor);
	
  	// render an image (lights and cameras are created automatically)
	renWin->Render();

  	// begin mouse interaction
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);
	// place at lower left corner
	AxesWidget->SetViewport( 0.0, 0.0, 0.25, 0.25 );
	AxesWidget->SetOrientationMarker( AxesActor );
	AxesWidget->KeyPressActivationOff();
	AxesWidget->SetInteractor(iren);
	AxesWidget->SetEnabled(1);
	AxesWidget->SetInteractive(0);
	AxesWidget->SetViewport(0., 0., 0.4, 0.4);

	iren->Start();
}
