/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_subsample_grid.cpp
	Purpose:
		Ported from SubsampleGrid.tcl
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkDataSetMapper.h"
#include "vtkExtractGrid.h"
#include "vtkPLOT3DReader.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkStructuredGridOutlineFilter.h"


#include "vtkProperty.h"
#include "vtkCamera.h"


// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	// This example demonstrates the subsampling of a structured grid.



	// Read some structured data.
	//
	vtkPLOT3DReader *pl3d = vtkPLOT3DReader::New();
	pl3d->SetXYZFileName(VTK_DATA_ROOT "combxyz.bin");
	pl3d->SetQFileName(VTK_DATA_ROOT "combq.bin");
	pl3d->SetScalarFunctionNumber(100);
	pl3d->SetVectorFunctionNumber(202);
	pl3d->Update();

	// Here we subsample the grid. The SetVOI method requires six values
	// specifying (imin,imax, jmin,jmax, kmin,kmax) extents. In this example
	// we extracting a plane. Note that the VOI is clamped to zero (min) and
	// the maximum i-j-k value; that way we can use the -1000,1000 specification
	// and be sure the values are clamped. The SampleRate specifies that we take
	// every point in the i-direction; every other point in the j-direction; and
	// every third point in the k-direction. IncludeBoundaryOn makes sure that we
	// get the boundary points even if the SampleRate does not coincident with
	// the boundary.
	//
	vtkExtractGrid *extract = vtkExtractGrid::New();
	extract->SetInputConnection(pl3d->GetOutputPort());
	extract->SetVOI(30, 30, -1000, 1000, -1000, 1000);
	extract->SetSampleRate(1, 2, 3);
	extract->IncludeBoundaryOn();
	vtkDataSetMapper *mapper = vtkDataSetMapper::New();
	mapper->SetInputConnection(extract->GetOutputPort());
	mapper->SetScalarRange(.18, .7);
	vtkActor *actor = vtkActor::New();
	actor->SetMapper(mapper);

	vtkStructuredGridOutlineFilter *outline = vtkStructuredGridOutlineFilter::New();
	outline->SetInputConnection(pl3d->GetOutputPort());
	vtkPolyDataMapper *outlineMapper = vtkPolyDataMapper::New();
	outlineMapper->SetInputConnection(outline->GetOutputPort());
	vtkActor *outlineActor = vtkActor::New();
	outlineActor->SetMapper(outlineMapper);
	outlineActor->GetProperty()->SetColor(0,0, 0);

	// Add the usual rendering stuff.
	//
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//
	ren1->AddActor(outlineActor);
	ren1->AddActor(actor);

	ren1->SetBackground(1, 1, 1);
	renWin->SetSize(300, 180);

	vtkCamera* cam1 = (ren1->GetActiveCamera());
	cam1->SetClippingRange(2.64586, 47.905);
	cam1->SetFocalPoint(8.931, 0.358127, 31.3526);
	cam1->SetPosition(29.7111, -0.688615, 37.1495);
	cam1->SetViewUp(-0.268328, 0.00801595, 0.963294);

	// render the image
	//

	renWin->Render();


	iren->Initialize();
	iren->Start();


	return 0 ;
}