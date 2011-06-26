/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File: demo_vtk_ice_cream.cpp
	Purpose:
         Ported from iceCream.tcl
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkCone.h"
#include "vtkContourFilter.h"
#include "vtkImplicitBoolean.h"
#include "vtkPlane.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSampleFunction.h"
#include "vtkSphere.h"
#include "vtkProperty.h"
#include "vtkCamera.h"

// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"

int main(int argc, char** argv)
{
	// Create implicit function primitives. These have been carefully placed to
	// give the effect that we want. We are going to use various combinations of
	// these functions to create the shape we want; for example, we use planes
	// intersected with a cone (which is infinite in extent) to get a finite
	// cone.
	//
	vtkCone *cone = vtkCone::New();
	cone->SetAngle(20);
	vtkPlane *vertPlane = vtkPlane::New();
	vertPlane->SetOrigin(.1, 0, 0);
	vertPlane->SetNormal(-1, 0, 0);
	vtkPlane *basePlane = vtkPlane::New();
	basePlane->SetOrigin(1.2, 0, 0);
	basePlane->SetNormal(1, 0, 0);
	vtkSphere *iceCream = vtkSphere::New();
	iceCream->SetCenter(1.333, 0, 0);
	iceCream->SetRadius(0.5);
	vtkSphere *bite = vtkSphere::New();
	bite->SetCenter(1.5, 0, 0.5);
	bite->SetRadius(0.25);

	// Combine primitives to build ice-cream cone. Clip the cone with planes.
	vtkImplicitBoolean *theCone = vtkImplicitBoolean::New();
	theCone->SetOperationTypeToIntersection();
	theCone->AddFunction(cone);
	theCone->AddFunction(vertPlane);
	theCone->AddFunction(basePlane);

	// Take a bite out of the ice cream.
	vtkImplicitBoolean *theCream = vtkImplicitBoolean::New();
	theCream->SetOperationTypeToDifference();
	theCream->AddFunction(iceCream);
	theCream->AddFunction(bite);

	// The sample function generates a distance function from the
	// implicit function (which in this case is the cone). This is
	// then contoured to get a polygonal surface.
	//
	vtkSampleFunction *theConeSample = vtkSampleFunction::New();
	theConeSample->SetImplicitFunction(theCone);
	theConeSample->SetModelBounds(-1, 1.5, -1.25, 1.25, -1.25, 1.25);
	theConeSample->SetSampleDimensions(60, 60, 60);
	theConeSample->ComputeNormalsOff();
	vtkContourFilter *theConeSurface = vtkContourFilter::New();
	theConeSurface->SetInputConnection(theConeSample->GetOutputPort());
	theConeSurface->SetValue(0, 0.0);
	vtkPolyDataMapper *coneMapper = vtkPolyDataMapper::New();
	coneMapper->SetInputConnection(theConeSurface->GetOutputPort());
	coneMapper->ScalarVisibilityOff();
	vtkActor *coneActor = vtkActor::New();
	coneActor->SetMapper(coneMapper);
	coneActor->GetProperty()->SetColor(1.0, 1.0, 0.0);

	// The same here for the ice cream.
	//
	vtkSampleFunction *theCreamSample = vtkSampleFunction::New();
	theCreamSample->SetImplicitFunction(theCream);

	theCreamSample->SetModelBounds( 0, 2.5, -1.25, 1.25, -1.25, 1.25);
	theCreamSample->SetSampleDimensions(60, 60, 60);
	theCreamSample->ComputeNormalsOff();
	vtkContourFilter *theCreamSurface = vtkContourFilter::New();
	theCreamSurface->SetInputConnection(theCreamSample->GetOutputPort());
	theCreamSurface->SetValue(0, 0.0);
	vtkPolyDataMapper *creamMapper = vtkPolyDataMapper::New();
	creamMapper->SetInputConnection(theCreamSurface->GetOutputPort());
	creamMapper->ScalarVisibilityOff();
	vtkActor *creamActor = vtkActor::New();
	creamActor->SetMapper(creamMapper);
	creamActor->GetProperty()->SetColor(0.0,1.0,1.0);

	// Create the usual rendering stuff
	//
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//
	ren1->AddActor(coneActor);
	ren1->AddActor(creamActor);
	ren1->SetBackground(1, 1, 1);
	renWin->SetSize(250, 250);
	ren1->ResetCamera();
	ren1->GetActiveCamera()->Roll(90);
	ren1->GetActiveCamera()->Dolly(1.5);
	ren1->ResetCameraClippingRange();
	iren->Initialize();

	iren->Start();


	return 0 ;
}