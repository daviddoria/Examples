/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_stl_reader.cpp
	Purpose:
		Ported from stl.tcl
*/
#include "project_config.h"

#include "vtkLODActor.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSTLReader.h"

#include "vtkProperty.h"
#include "vtkCamera.h"


// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	// This example demonstrates the use of vtkSTLReader to load data into VTK from
	// a file.  This example also uses vtkLODActor which changes its graphical
	// representation of the data to maintain interactive performance.

	//
	// First we include the VTK Tcl packages which will make available
	// all of the vtk commands to Tcl
	//

	// Create the reader and read a data file.  Connect the mapper and actor.
	vtkSTLReader *sr = vtkSTLReader::New();
	sr->SetFileName(VTK_DATA_ROOT "42400-IDGH.stl");

	vtkPolyDataMapper *stlMapper = vtkPolyDataMapper::New();
	stlMapper->SetInputConnection(sr->GetOutputPort());

	vtkLODActor *stlActor = vtkLODActor::New();
	stlActor->SetMapper(stlMapper);

	// Create the Renderer, RenderWindow, and RenderWindowInteractor
	//
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the render; set the background and size
	//
	ren1->AddActor(stlActor);
	ren1->SetBackground(0.1, 0.2, 0.4);
	renWin->SetSize(500, 500);

	// Zoom in closer
	ren1->ResetCamera();
	vtkCamera* cam1 = (ren1->GetActiveCamera());
	cam1->Zoom(1.4);


	iren->Initialize();
	iren->Start();

	return 0 ;
}