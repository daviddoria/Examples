/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_contour2D.cpp
	Purpose:
		Ported from Contours2D.tcl
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkContourFilter.h"
#include "vtkExtractVOI.h"
#include "vtkOutlineFilter.h"
#include "vtkPolyDataMapper.h"
#include "vtkQuadric.h"
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
	// This example shows how to sample a mathematical function over a
	// volume. A slice from the volume is then extracted and then contoured
	// to produce 2D contour lines.
	//

	// Quadric definition. This is a type of implicit function. Here the
	// coefficients to the equations are set.
	vtkQuadric *quadric = vtkQuadric::New();
	quadric->SetCoefficients(.5, 1, .2, 0, .1, 0, 0, .2, 0, 0);

	// The vtkSampleFunction uses the quadric function and evaluates function
	// value over a regular lattice (i.e., a volume).
	vtkSampleFunction *sample = vtkSampleFunction::New();
	sample->SetSampleDimensions(30, 30, 30);
	sample->SetImplicitFunction(quadric);
	sample->ComputeNormalsOff();

	// Here a single slice (i.e., image) is extracted from the volume. (Note: in
	// actuality the VOI request causes the sample function to operate on just the
	// slice.)
	vtkExtractVOI *extract = vtkExtractVOI::New();
	extract->SetInputConnection(sample->GetOutputPort());
	extract->SetVOI(0, 29, 0, 29, 15, 15);
	extract->SetSampleRate(1, 2, 3);

	// The image is contoured to produce contour lines. Thirteen contour values
	// ranging from (0,1.2) inclusive are produced.
	vtkContourFilter *contours = vtkContourFilter::New();
	contours->SetInputConnection(extract->GetOutputPort());
	contours->GenerateValues(13, 0.0, 1.2);

	// The contour lines are mapped to the graphics library.
	vtkPolyDataMapper *contMapper = vtkPolyDataMapper::New();
	contMapper->SetInputConnection(contours->GetOutputPort());
	contMapper->SetScalarRange(0.0, 1.2);

	vtkActor *contActor = vtkActor::New();
	contActor->SetMapper(contMapper);

	// Create outline an outline of the sampled data.
	vtkOutlineFilter *outline = vtkOutlineFilter::New();
	outline->SetInputConnection(sample->GetOutputPort());

	vtkPolyDataMapper *outlineMapper = vtkPolyDataMapper::New();
	outlineMapper->SetInputConnection(outline->GetOutputPort());

	vtkActor *outlineActor = vtkActor::New();
	outlineActor->SetMapper(outlineMapper);
	outlineActor->GetProperty()->SetColor(0, 0, 0);

	// Create the renderer, render window, and interactor.
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Set the background color to white. Associate the actors with the
	// renderer.
	ren1->SetBackground(1, 1, 1);
	ren1->AddActor(contActor);
	ren1->AddActor(outlineActor);


	ren1->ResetCamera();
	ren1->GetActiveCamera()->Zoom(1.5);

	iren->Initialize();
	iren->Start();

	return 0 ;
}