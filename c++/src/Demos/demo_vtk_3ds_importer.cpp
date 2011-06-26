/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_3ds_importer.cpp
	Purpose:
		Ported from flamingo.tcl
*/
#include "project_config.h"

#include "vtk3DSImporter.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"

#include "vtkProperty.h"
#include "vtkCamera.h"


// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	// This example demonstrates the use of vtk3DSImporter.
	// vtk3DSImporter is used to load 3D Studio files. Unlike writers,
	// importers can load scenes (data as well as lights, cameras, actors
	// etc.). Importers will either generate an instance of vtkRenderWindow
	// and/or vtkRenderer or will use the ones you specify.

	//
	// First we include the VTK Tcl packages which will make available
	// all of the vtk commands to Tcl
	//


	// Create the importer and read a file
	vtk3DSImporter *importer = vtk3DSImporter::New();
	importer->ComputeNormalsOn();
	importer->SetFileName(VTK_DATA_ROOT "iflamigm.3ds");
	importer->Read();
	// Here we let the importer create a renderer and a render window
	// for us. We could have also create and assigned those ourselves:
	// vtkRenderWindow renWin
	// importer SetRenderWindow renWin

	// Assign an interactor.
	// We have to ask the importer for it's render window.
	vtkRenderWindow* renWin = (importer->GetRenderWindow());
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Set the render window's size
	renWin->SetSize(300, 300);

	// Set some properties on the renderer.
	// We have to ask the importer for it's renderer.
	vtkRenderer* ren = (importer->GetRenderer());
	ren->SetBackground(0.1, 0.2, 0.4);

	// Position the camera:
	// change view up to +z
	vtkCamera* camera = (ren->GetActiveCamera());
	camera->SetPosition(0, 1, 0);
	camera->SetFocalPoint(0, 0, 0);
	camera->SetViewUp(0, 0, 1);
	// let the renderer compute good position and focal point
	ren->ResetCamera();
	camera->Dolly(1.4);
	ren->ResetCameraClippingRange();



	iren->Initialize();
	iren->Start();

	return 0 ;
}