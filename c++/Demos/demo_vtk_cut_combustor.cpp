/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_cut_combustor.cpp
	Purpose:
		Ported from CutCombustor.tcl
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkCutter.h"
#include "vtkPLOT3DReader.h"
#include "vtkPlane.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkStructuredGridGeometryFilter.h"
#include "vtkStructuredGridOutlineFilter.h"
#include "vtkStructuredGrid.h"
#include "vtkPointData.h"

#include "vtkProperty.h"
#include "vtkCamera.h"


// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	// This example shows how to use cutting (vtkCutter) and how it compares
	// with extracting a plane from a computational grid.
	//

	// Read some data.
	vtkPLOT3DReader *pl3d = vtkPLOT3DReader::New();
	pl3d->SetXYZFileName(VTK_DATA_ROOT "combxyz.bin");
	pl3d->SetQFileName( VTK_DATA_ROOT "combq.bin");
	pl3d->SetScalarFunctionNumber(100);
	pl3d->SetVectorFunctionNumber(202);
	pl3d->Update();

	// The cutter uses an implicit function to perform the cutting.
	// Here we define a plane, specifying its center and normal.
	// Then we assign the plane to the cutter.
	vtkPlane *plane = vtkPlane::New();

	plane->SetOrigin(pl3d->GetOutput()->GetCenter());
	plane->SetNormal(-0.287, 0, 0.9579);
	vtkCutter *planeCut = vtkCutter::New();
	planeCut->SetInputConnection(pl3d->GetOutputPort());
	planeCut->SetCutFunction(plane);
	vtkPolyDataMapper *cutMapper = vtkPolyDataMapper::New();
	cutMapper->SetInputConnection(planeCut->GetOutputPort());
    cutMapper->SetScalarRange( pl3d->GetOutput()->GetPointData()->GetScalars()->GetRange() );
	vtkActor *cutActor = vtkActor::New();
	cutActor->SetMapper(cutMapper);

	// Here we extract a computational plane from the structured grid.
	// We render it as wireframe.
	vtkStructuredGridGeometryFilter *compPlane = vtkStructuredGridGeometryFilter::New();
	compPlane->SetInputConnection(pl3d->GetOutputPort());
	compPlane->SetExtent(0, 100, 0, 100, 9, 9);
	vtkPolyDataMapper *planeMapper = vtkPolyDataMapper::New();
	planeMapper->SetInputConnection(compPlane->GetOutputPort());
	planeMapper->ScalarVisibilityOff();
	vtkActor *planeActor = vtkActor::New();
	planeActor->SetMapper(planeMapper);

	planeActor->GetProperty()->SetColor(0, 0, 0);

	// The outline of the data puts the data in context.
	vtkStructuredGridOutlineFilter *outline = vtkStructuredGridOutlineFilter::New();
	outline->SetInputConnection(pl3d->GetOutputPort());
	vtkPolyDataMapper *outlineMapper = vtkPolyDataMapper::New();
	outlineMapper->SetInputConnection(outline->GetOutputPort());
	vtkActor *outlineActor = vtkActor::New();
	outlineActor->SetMapper(outlineMapper);

	vtkProperty* outlineProp = (outlineActor->GetProperty());
	outlineProp->SetColor( 0, 0, 0);

	// Create the RenderWindow, Renderer and both Actors
	//
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//
	ren1->AddActor(outlineActor);
	ren1->AddActor(planeActor);
	ren1->AddActor(cutActor);

	ren1->SetBackground(1, 1, 1);
	renWin->SetSize(400, 300);

	vtkCamera* cam1 = (ren1->GetActiveCamera());
	cam1->SetClippingRange(11.1034, 59.5328);
	cam1->SetFocalPoint(9.71821, 0.458166, 29.3999);
	cam1->SetPosition(-2.95748, -26.7271, 44.5309);
	cam1->SetViewUp(0.0184785, 0.479657, 0.877262);
	iren->Initialize();
	iren->Start();
	return 0 ;
}