/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_PerlinTerrain.cpp
	Purpose:
		Ported from PerlinTerrain.tcl
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkContourFilter.h"
#include "vtkImplicitSum.h"
#include "vtkPerlinNoise.h"
#include "vtkPlane.h"
#include "vtkPolyDataMapper.h"
#include "vtkPolyDataNormals.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSampleFunction.h"


#include "vtkProperty.h"
#include "vtkCamera.h"

// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	vtkPlane *plane = vtkPlane::New();

	vtkPerlinNoise *p1 = vtkPerlinNoise::New();
	p1->SetFrequency(1, 1, 0);

	vtkPerlinNoise *p2 = vtkPerlinNoise::New();
	p2->SetFrequency(3, 5, 0);
	p2->SetPhase(0.5, 0.5, 0);

	vtkPerlinNoise *p3 = vtkPerlinNoise::New();
	p3->SetFrequency(16, 16, 0);

	vtkImplicitSum *sum = vtkImplicitSum::New();
	sum->SetNormalizeByWeight(1);
	sum->AddFunction(plane);
	sum->AddFunction(p1, 0.2);
	sum->AddFunction(p2, 0.1);
	sum->AddFunction(p3, 0.02);

	vtkSampleFunction *sample = vtkSampleFunction::New();
	sample->SetImplicitFunction(sum);
	sample->SetSampleDimensions(65, 65, 20);
	sample->SetModelBounds(-1, 1, -1, 1, -0.5, 0.5);
	sample->ComputeNormalsOff();
	vtkContourFilter *surface = vtkContourFilter::New();
	surface->SetInputConnection(sample->GetOutputPort());
	surface->SetValue(0, 0.0);

	vtkPolyDataNormals *smooth = vtkPolyDataNormals::New();
	smooth->SetInputConnection(surface->GetOutputPort());
	smooth->SetFeatureAngle(90);

	vtkPolyDataMapper *mapper = vtkPolyDataMapper::New();
	mapper->SetInputConnection(smooth->GetOutputPort());
	mapper->ScalarVisibilityOff();
	vtkActor *actor = vtkActor::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(0.4, 0.2, 0.1);

	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//
	ren1->AddActor(actor);
	ren1->SetBackground(1, 1, 1);
	renWin->SetSize(500, 500);
	ren1->ResetCamera();
	ren1->GetActiveCamera()->Elevation(-45);
	ren1->GetActiveCamera()->Azimuth(10);
	ren1->GetActiveCamera()->Dolly(1.35);
	ren1->ResetCameraClippingRange();
	iren->Initialize();
	iren->Start();



	return 0 ;
}