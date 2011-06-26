/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_plane_texture.cpp
	Purpose:
		Ported from TPlane.tcl
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkBMPReader.h"
#include "vtkPlaneSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkTexture.h"

#include "vtkProperty.h"
#include "vtkCamera.h"


// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	//
	// This simple example shows how to do basic texture mapping.
	//


	// Load in the texture map. A texture is any unsigned char image. If it
	// is not of this type, you will have to map it through a lookup table
	// or by using vtkImageShiftScale.
	//
	vtkBMPReader *bmpReader = vtkBMPReader::New();
	bmpReader->SetFileName(VTK_DATA_ROOT "masonry.bmp");
	vtkTexture *atext = vtkTexture::New();
	atext->SetInputConnection(bmpReader->GetOutputPort());
	atext->InterpolateOn();

	// Create a plane source and actor. The vtkPlanesSource generates
	// texture coordinates.
	//
	vtkPlaneSource *plane = vtkPlaneSource::New();
	vtkPolyDataMapper *planeMapper = vtkPolyDataMapper::New();
	planeMapper->SetInputConnection(plane->GetOutputPort());
	vtkActor *planeActor = vtkActor::New();
	planeActor->SetMapper(planeMapper);
	planeActor->SetTexture(atext);

	// Create the RenderWindow, Renderer and both Actors
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	ren1->AddActor(planeActor);
	ren1->SetBackground(0.1, 0.2, 0.4);
	renWin->SetSize(500, 500);

	// render the image
	renWin->Render();

	ren1->ResetCamera();
	vtkCamera* cam1 = (ren1->GetActiveCamera());
	cam1->Elevation(-30);
	cam1->Roll(-20);
	ren1->ResetCameraClippingRange();
	renWin->Render();


	iren->Initialize();
	iren->Start();



	return 0 ;
}