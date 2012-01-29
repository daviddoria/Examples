/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_image_wrap.cpp
	Purpose:
		Ported from imageWarp.tcl
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkBMPReader.h"
#include "vtkDataSetMapper.h"
#include "vtkImageDataGeometryFilter.h"
#include "vtkImageLuminance.h"
#include "vtkMergeFilter.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkWarpScalar.h"



#include "vtkProperty.h"
#include "vtkCamera.h"


// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	// This example shows how to combine data from both the imaging
	// and graphics pipelines. The vtkMergeData filter is used to
	// merge the data from each together.

	// Read in an image and compute a luminance value. The image is extracted
	// as a set of polygons (vtkImageDataGeometryFilter). We then will
	// warp the plane using the scalar (luminance) values.
	//
	vtkBMPReader *reader = vtkBMPReader::New();
	reader->SetFileName( VTK_DATA_ROOT "masonry.bmp" );
	vtkImageLuminance *luminance = vtkImageLuminance::New();
	luminance->SetInputConnection(reader->GetOutputPort());
	vtkImageDataGeometryFilter *geometry = vtkImageDataGeometryFilter::New();
	geometry->SetInputConnection(luminance->GetOutputPort());
	vtkWarpScalar *warp = vtkWarpScalar::New();
	warp->SetInputConnection(geometry->GetOutputPort());
	warp->SetScaleFactor(-0.1);

	// Use vtkMergeFilter to combine the original image with the warped geometry.
	//
	vtkMergeFilter *merge = vtkMergeFilter::New();
	merge->SetGeometryConnection(warp->GetOutputPort());
	merge->SetScalarsConnection(reader->GetOutputPort());
	vtkDataSetMapper *mapper = vtkDataSetMapper::New();
	mapper->SetInputConnection(merge->GetOutputPort());
	mapper->SetScalarRange(0, 255);
	mapper->ImmediateModeRenderingOff();
	vtkActor *actor = vtkActor::New();
	actor->SetMapper(mapper);

	// Create renderer stuff
	//
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//
	ren1->AddActor(actor);
	ren1->ResetCamera();
	ren1->GetActiveCamera()->Azimuth(20);
	ren1->GetActiveCamera()->Elevation(30);
	ren1->SetBackground(0.1, 0.2, 0.4);
	ren1->ResetCameraClippingRange();
	renWin->SetSize(250, 250);

	// render the image
	//

	vtkCamera* cam1 = (ren1->GetActiveCamera());
	cam1->Zoom(1.4);
	renWin->Render();


	iren->Initialize();
	iren->Start();


	return 0 ;
}