/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	   demo_vtk_procrustesAlignment.cpp
	Purpose:
	   Ported from procrustesAlignment.tcl

*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkPolyDataMapper.h"
#include "vtkProcrustesAlignmentFilter.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSphereSource.h"
#include "vtkTransform.h"
#include "vtkTransformPolyDataFilter.h"



#include "vtkProperty.h"
#include "vtkCamera.h"

// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	vtkSphereSource *sphere = vtkSphereSource::New();

	// make two copies of the shape and distort them a little

	vtkTransform *transform1 = vtkTransform::New();
	transform1->Translate(0.2, 0.1, 0.3);
	transform1->Scale(1.3, 1.1, 0.8);

	vtkTransform *transform2 = vtkTransform::New();
	transform2->Translate(0.3, 0.7, 0.1);
	transform2->Scale(1.0, 0.1, 1.8);

	vtkTransformPolyDataFilter *transformer1 = vtkTransformPolyDataFilter::New();
	transformer1->SetInputConnection(sphere->GetOutputPort());
	transformer1->SetTransform(transform1);

	vtkTransformPolyDataFilter *transformer2 = vtkTransformPolyDataFilter::New();
	transformer2->SetInputConnection(sphere->GetOutputPort());
	transformer2->SetTransform(transform2);

	// map these three shapes into the first renderer
	vtkPolyDataMapper *map1a = vtkPolyDataMapper::New();
	map1a->SetInputConnection(sphere->GetOutputPort());
	vtkActor *Actor1a = vtkActor::New();
	Actor1a->SetMapper(map1a);
	Actor1a->GetProperty()->SetDiffuseColor(1.0000, 0.3882, 0.2784);

	vtkPolyDataMapper *map1b = vtkPolyDataMapper::New();
	map1b->SetInputConnection(transformer1->GetOutputPort());
	vtkActor *Actor1b = vtkActor::New();
	Actor1b->SetMapper(map1b);
	Actor1b->GetProperty()->SetDiffuseColor(0.3882,1.0000, 0.2784);

	vtkPolyDataMapper *map1c = vtkPolyDataMapper::New();
	map1c->SetInputConnection(transformer2->GetOutputPort());
	vtkActor *Actor1c = vtkActor::New();
	Actor1c->SetMapper(map1c);
	Actor1c->GetProperty()->SetDiffuseColor(0.3882,0.2784, 1.0000);

	// -- align the shapes using Procrustes (using SetModeToRigidBody) --
	vtkProcrustesAlignmentFilter *procrustes1 = vtkProcrustesAlignmentFilter::New();
	procrustes1->SetNumberOfInputs(3);
	procrustes1->SetInput(0, sphere->GetOutput());
	procrustes1->SetInput(1, transformer1->GetOutput());
	procrustes1->SetInput(2, transformer2->GetOutput());
	;

	// map the aligned shapes into the second renderer
	vtkPolyDataMapper *map2a = vtkPolyDataMapper::New();
	map2a->SetInput((vtkPolyData*)procrustes1->GetOutput(0));
	vtkActor *Actor2a = vtkActor::New();
	Actor2a->SetMapper(map2a);
	Actor2a->GetProperty()->SetDiffuseColor(1.0000,0.3882, 0.2784);

	vtkPolyDataMapper *map2b = vtkPolyDataMapper::New();
	map2b->SetInput((vtkPolyData*)procrustes1->GetOutput(1));
	vtkActor *Actor2b = vtkActor::New();
	Actor2b->SetMapper(map2b);
	Actor2b->GetProperty()->SetDiffuseColor(0.3882,1.0000, 0.2784);

	vtkPolyDataMapper *map2c = vtkPolyDataMapper::New();
	map2c->SetInput((vtkPolyData*)procrustes1->GetOutput(2));
	vtkActor *Actor2c = vtkActor::New();
	Actor2c->SetMapper(map2c);
	Actor2c->GetProperty()->SetDiffuseColor(0.3882, 0.2784, 1.0000);

	// -- align the shapes using Procrustes (using SetModeToSimilarity (default)) --
	vtkProcrustesAlignmentFilter *procrustes2 = vtkProcrustesAlignmentFilter::New();
	procrustes2->SetNumberOfInputs(3);
	procrustes2->SetInput(0, sphere->GetOutput());
	procrustes2->SetInput(1, transformer1->GetOutput());
	procrustes2->SetInput(2, transformer2->GetOutput());

	// map the aligned shapes into the third renderer
	vtkPolyDataMapper *map3a = vtkPolyDataMapper::New();
	map3a->SetInput((vtkPolyData*)procrustes2->GetOutput(0));
	vtkActor *Actor3a = vtkActor::New();
	Actor3a->SetMapper(map3a);
	Actor3a->GetProperty()->SetDiffuseColor(1.0000,0.3882, 0.2784);

	vtkPolyDataMapper *map3b = vtkPolyDataMapper::New();
	map3b->SetInput((vtkPolyData*)procrustes2->GetOutput(1));
	vtkActor *Actor3b = vtkActor::New();
	Actor3b->SetMapper(map3b);
	Actor3b->GetProperty()->SetDiffuseColor(0.3882, 1.0000, 0.2784);

	vtkPolyDataMapper *map3c = vtkPolyDataMapper::New();
	map3c->SetInput((vtkPolyData*)procrustes2->GetOutput(2));
	vtkActor *Actor3c = vtkActor::New();
	Actor3c->SetMapper(map3c);
	Actor3c->GetProperty()->SetDiffuseColor(0.3882, 0.2784, 1.0000);

	// -- align the shapes using Procrustes (using SetModeToAffine) --
	vtkProcrustesAlignmentFilter *procrustes3 = vtkProcrustesAlignmentFilter::New();
	procrustes3->SetNumberOfInputs(3);
	procrustes3->SetInput(0, sphere->GetOutput());
	procrustes3->SetInput(1, transformer1->GetOutput());
	procrustes3->SetInput(2, transformer2->GetOutput());
	;

	// map the aligned shapes into the fourth renderer
	vtkPolyDataMapper *map4a = vtkPolyDataMapper::New();
	map4a->SetInput((vtkPolyData*)procrustes3->GetOutput(0));
	vtkActor *Actor4a = vtkActor::New();
	Actor4a->SetMapper(map4a);
	Actor4a->GetProperty()->SetDiffuseColor(1.0000,0.3882, 0.2784);

	vtkPolyDataMapper *map4b = vtkPolyDataMapper::New();
	map4b->SetInput((vtkPolyData*)procrustes3->GetOutput(1));
	vtkActor *Actor4b = vtkActor::New();
	Actor4b->SetMapper(map4b);
	Actor4b->GetProperty()->SetDiffuseColor(0.3882,1.0000, 0.2784);

	vtkPolyDataMapper *map4c = vtkPolyDataMapper::New();
	map4c->SetInput((vtkPolyData*)procrustes3->GetOutput(2));
	vtkActor *Actor4c = vtkActor::New();
	Actor4c->SetMapper(map4c);
	Actor4c->GetProperty()->SetDiffuseColor(0.3882,0.2784, 1.0000);

	// Create the RenderWindow and its four Renderers

	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderer *ren2 = vtkRenderer::New();
	vtkRenderer *ren3 = vtkRenderer::New();
	vtkRenderer *ren4 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	renWin->AddRenderer(ren2);
	renWin->AddRenderer(ren3);
	renWin->AddRenderer(ren4);
	renWin->SetSize(400, 100);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer

	ren1->AddActor(Actor1a);
	ren1->AddActor(Actor1b);
	ren1->AddActor(Actor1c);

	ren2->AddActor(Actor2a);
	ren2->AddActor(Actor2b);
	ren2->AddActor(Actor2c);

	ren3->AddActor(Actor3a);
	ren3->AddActor(Actor3b);
	ren3->AddActor(Actor3c);

	ren4->AddActor(Actor4a);
	ren4->AddActor(Actor4b);
	ren4->AddActor(Actor4c);

	// set the properties of the renderers

	ren1->SetBackground(1, 1, 1);
	ren1->SetViewport(0.0, 0.0, 0.25, 1.0);
	ren1->GetActiveCamera()->SetPosition(1, -1, 0);
	ren1->ResetCamera();

	ren2->SetBackground(1, 1, 1);
	ren2->SetViewport(0.25, 0.0, 0.5, 1.0);
	ren2->GetActiveCamera()->SetPosition(1, -1, 0 );
	ren2->ResetCamera();

	ren3->SetBackground(1, 1, 1);
	ren3->SetViewport(0.5, 0.0, 0.75, 1.0);
	ren3->GetActiveCamera()->SetPosition(1, -1, 0);
	ren3->ResetCamera();

	ren4->SetBackground(1, 1, 1);
	ren4->SetViewport(0.75, 0.0, 1.0, 1.0);
	ren4->GetActiveCamera()->SetPosition(1, -1, 0);
	ren4->ResetCamera();

	// render the image
	//

	renWin->Render();

	iren->Initialize();
	iren->Start();



	return 0 ;
}