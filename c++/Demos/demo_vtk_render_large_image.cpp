/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_render_large_image.cpp
	Purpose:
		Ported from RenderLargeImage.tcl
*/
#include "project_config.h"

#include "vtk3DSImporter.h"
#include "vtkRenderLargeImage.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkTIFFWriter.h"

#include "vtkProperty.h"
#include "vtkCamera.h"


// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	//
	// This simple example shows how to render a very large image (i.e., one
	// that cannot fit on the screen).
	//


	// We'll import some data to start. Since we are using an importer, we've
	// got to give it a render window and such. Note that the render window
	// size is set fairly small.
	vtkRenderer *ren = vtkRenderer::New();
	ren->SetBackground(0.1, 0.2, 0.4);
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren);
	renWin->SetSize(125, 125);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	vtk3DSImporter *importer = vtk3DSImporter::New();
	importer->SetRenderWindow(renWin);
	importer->SetFileName(VTK_DATA_ROOT "Viewpoint\\iflamigm.3ds");
	importer->ComputeNormalsOn();
	importer->Read();

	// We'll set up the view we want.
	//
	ren->GetActiveCamera()->SetPosition(0, 1, 0);
	ren->GetActiveCamera()->SetFocalPoint(0,0, 0);
	ren->GetActiveCamera()->SetViewUp(0,0, 1);

	// Let the renderer compute a good position and focal point.
	//
	ren->ResetCamera();
	ren->GetActiveCamera()->Dolly(1.4);
	ren->ResetCameraClippingRange();


	// render the large image

	// Here we request that the large image is four times bigger than the
	// renderers image.
	//
	vtkRenderLargeImage *renderLarge = vtkRenderLargeImage::New();
	renderLarge->SetInput(ren);
	renderLarge->SetMagnification(4);

	// We write out the image which causes the rendering to occur. If you
	// watch your screen you will see the pieces being rendered right after
	// one another.
	//
	vtkTIFFWriter *writer = vtkTIFFWriter::New();
	writer->SetInputConnection(renderLarge->GetOutputPort());
	writer->SetFileName("largeImage.tif");
	writer->Write();


	printf("Done writting...");
	iren->Start();



	return 0 ;
}