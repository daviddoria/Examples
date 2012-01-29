/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_streamline.cpp
	Purpose:
		Ported from streamSurface.tcl
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkLineSource.h"
#include "vtkPLOT3DReader.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkRuledSurfaceFilter.h"
#include "vtkRungeKutta4.h"
#include "vtkStreamTracer.h"
#include "vtkStructuredGridOutlineFilter.h"
#include "vtkStructuredGrid.h"

#include "vtkProperty.h"
#include "vtkCamera.h"


// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	// This example demonstrates the generation of a streamsurface.


	// Read the data and specify which scalars and vectors to read.
	//
	vtkPLOT3DReader *pl3d = vtkPLOT3DReader::New();
	pl3d->SetXYZFileName(VTK_DATA_ROOT "combxyz.bin");
	pl3d->SetQFileName(VTK_DATA_ROOT "combq.bin");
	pl3d->SetScalarFunctionNumber(100);
	pl3d->SetVectorFunctionNumber(202);
	pl3d->Update();

	// We use a rake to generate a series of streamline starting points
	// scattered along a line. Each point will generate a streamline. These
	// streamlines are then fed to the vtkRuledSurfaceFilter which stitches
	// the lines together to form a surface.
	//
	vtkLineSource *rake = vtkLineSource::New();
	rake->SetPoint1(15, -5, 32);
	rake->SetPoint2(15, 5, 32);
	rake->SetResolution(21);
	vtkPolyDataMapper *rakeMapper = vtkPolyDataMapper::New();
	rakeMapper->SetInputConnection(rake->GetOutputPort());
	vtkActor *rakeActor = vtkActor::New();
	rakeActor->SetMapper(rakeMapper);

	vtkRungeKutta4 *integ = vtkRungeKutta4::New();
	vtkStreamTracer *sl = vtkStreamTracer::New();
	sl->SetInputConnection(pl3d->GetOutputPort());
	sl->SetSourceConnection(rake->GetOutputPort());
	sl->SetIntegrator(integ);
	sl->SetMaximumPropagation(0.1);
	sl->SetMaximumPropagationUnitToTimeUnit();
	sl->SetInitialIntegrationStep(0.1);
	sl->SetInitialIntegrationStepUnitToCellLengthUnit();
	sl->SetIntegrationDirectionToBackward();

	//
	// The ruled surface stiches together lines with triangle strips.
	// Note the SetOnRatio method. It turns on every other strip that
	// the filter generates (only when multiple lines are input).
	//
	vtkRuledSurfaceFilter *scalarSurface = vtkRuledSurfaceFilter::New();
	scalarSurface->SetInputConnection(sl->GetOutputPort());
	scalarSurface->SetOffset(0);
	scalarSurface->SetOnRatio(2);
	scalarSurface->PassLinesOn();
	scalarSurface->SetRuledModeToPointWalk();
	scalarSurface->SetDistanceFactor(30);
	vtkPolyDataMapper *mapper = vtkPolyDataMapper::New();
	mapper->SetInputConnection(scalarSurface->GetOutputPort());
	mapper->SetScalarRange(pl3d->GetOutput()->GetScalarRange());

	vtkActor *actor = vtkActor::New();
	actor->SetMapper(mapper);

	// Put an outline around for context.
	//
	vtkStructuredGridOutlineFilter *outline = vtkStructuredGridOutlineFilter::New();
	outline->SetInputConnection(pl3d->GetOutputPort());
	vtkPolyDataMapper *outlineMapper = vtkPolyDataMapper::New();
	outlineMapper->SetInputConnection(outline->GetOutputPort());
	vtkActor *outlineActor = vtkActor::New();
	outlineActor->SetMapper(outlineMapper);
	outlineActor->GetProperty()->SetColor(0, 0, 0);

	// Now create the usual graphics stuff.
	vtkRenderer *ren = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	ren->AddActor(rakeActor);
	ren->AddActor(actor);
	ren->AddActor(outlineActor);
	ren->SetBackground(1, 1, 1);

	renWin->SetSize(300, 300);


	iren->Initialize();
	iren->Start();


	return 0 ;
}