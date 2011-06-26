/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File: demo_vtk_hello.cpp
	Purpose:
		Ported from hello.tcl (Not work for debug mode)
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkCamera.h"
#include "vtkContourFilter.h"
#include "vtkImplicitModeller.h"
#include "vtkPolyDataMapper.h"
#include "vtkPolyDataReader.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"

#include "vtkProperty.h"
#include "vtkCamera.h"

// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	// Create lines which serve as the "seed" geometry. The lines spell the
	// word "hello".
	//
	vtkPolyDataReader *reader = vtkPolyDataReader::New();
	reader->SetFileName(VTK_DATA_ROOT "hello.vtk");
	vtkPolyDataMapper *lineMapper = vtkPolyDataMapper::New();
	lineMapper->SetInputConnection(reader->GetOutputPort());
	vtkActor *lineActor = vtkActor::New();
	lineActor->SetMapper(lineMapper);
	lineActor->GetProperty()->SetColor(1,0,0);

	// Create implicit model with vtkImplicitModeller. This computes a scalar
	// field which is the distance from the generating geometry. The contour
	// filter then extracts the geoemtry at the distance value 0.25 from the
	// generating geometry.
	//
	vtkImplicitModeller *imp = vtkImplicitModeller::New();
	imp->SetInputConnection(reader->GetOutputPort());
	imp->SetSampleDimensions(110, 40, 20);
	imp->SetMaximumDistance(0.25);
	imp->SetModelBounds(-1.0, 10.0, -1.0, 3.0, -1.0, 1.0);
	vtkContourFilter *contour = vtkContourFilter::New();
	contour->SetInputConnection(imp->GetOutputPort());
	contour->SetValue(0, 0.25);
	vtkPolyDataMapper *impMapper = vtkPolyDataMapper::New();
	impMapper->SetInputConnection(contour->GetOutputPort());
	impMapper->ScalarVisibilityOff();
	vtkActor *impActor = vtkActor::New();
	impActor->SetMapper(impMapper);
	impActor->GetProperty()->SetColor(0.5,0,0);
	impActor->GetProperty()->SetOpacity(0.5);

	// Create the usual graphics stuff.
	// Create the RenderWindow, Renderer and both Actors
	//
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//
	ren1->AddActor(lineActor);
	ren1->AddActor(impActor);
	ren1->SetBackground(1, 1, 1);
	renWin->SetSize(600, 250);

	vtkCamera *camera = vtkCamera::New();
	camera->SetClippingRange(1.81325, 90.6627);
	camera->SetFocalPoint( 4.5,  1,  0);

	camera->SetPosition( 4.5, 1.0, 6.73257);
	camera->SetViewUp( 0,  1 , 0);
	camera->Zoom(0.8);
	ren1->SetActiveCamera(camera);

	iren->Initialize();
	iren->Start();



	return 0 ;
}