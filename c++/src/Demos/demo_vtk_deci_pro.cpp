/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_deci_pro.cpp
	Purpose:
		Ported from deciFran.tcl
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkCamera.h"
#include "vtkDecimatePro.h"
#include "vtkPolyDataMapper.h"
#include "vtkPolyDataNormals.h"
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
	// This example shows how to use decimation to reduce a polygonal mesh. We also
	// use mesh smoothing and generate surface normals to give a pleasing result.
	//

	// We start by reading some data that was originally captured from
	// a Cyberware laser digitizing system.
	//
	vtkPolyDataReader *fran = vtkPolyDataReader::New();
	fran->SetFileName(VTK_DATA_ROOT "fran_cut.vtk");

	// We want to preserve topology (not let any cracks form). This may limit
	// the total reduction possible, which we have specified at 90%.
	//
	vtkDecimatePro *deci = vtkDecimatePro::New();
	deci->SetInputConnection(fran->GetOutputPort());
	deci->SetTargetReduction(0.9);
	deci->PreserveTopologyOn();
	vtkPolyDataNormals *normals = vtkPolyDataNormals::New();
	normals->SetInputConnection(fran->GetOutputPort());
	normals->FlipNormalsOn();
	vtkPolyDataMapper *franMapper = vtkPolyDataMapper::New();
	franMapper->SetInputConnection(normals->GetOutputPort());
	vtkActor *franActor = vtkActor::New();
	franActor->SetMapper(franMapper);
	franActor->GetProperty()->SetColor(1.0, 0.49, 0.25);


	// Create the RenderWindow, Renderer and both Actors
	//
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//
	ren1->AddActor(franActor);
	ren1->SetBackground(1, 1, 1);
	renWin->SetSize(250, 250);

	// render the image
	//


	vtkCamera *cam1 = vtkCamera::New();
	cam1->SetClippingRange(0.0475572, 2.37786);
	cam1->SetFocalPoint(0.052665, -0.129454, -0.0573973);
	cam1->SetPosition(0.327637, -0.116299, -0.256418);
	cam1->SetViewUp(-0.0225386, 0.999137, 0.034901);
	ren1->SetActiveCamera(cam1);

	iren->Initialize();


	iren->Start();


	return 0 ;
}