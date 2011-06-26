/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_simple_texturemap2d.cpp
	Purpose:
		Ported from SimpleTextureMap2D.tcl
*/
#include "project_config.h"

#include "vtkColorTransferFunction.h"
#include "vtkPiecewiseFunction.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkStructuredPointsReader.h"
#include "vtkVolume.h"
#include "vtkVolumeProperty.h"
#include "vtkVolumeTextureMapper2D.h"



#include "vtkProperty.h"
#include "vtkCamera.h"

// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	// This is a simple volume rendering example that
	// uses a vtkVolumeRayCast mapper


	// Create the standard renderer, render window
	// and interactor
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Create the reader for the data
	vtkStructuredPointsReader *reader = vtkStructuredPointsReader::New();
	reader->SetFileName(VTK_DATA_ROOT "ironProt.vtk");

	// Create transfer mapping scalar value to opacity
	vtkPiecewiseFunction *opacityTransferFunction = vtkPiecewiseFunction::New();
	opacityTransferFunction->AddPoint( 20 , 0.0);
	opacityTransferFunction->AddPoint( 255 , 0.2);

	// Create transfer mapping scalar value to color
	vtkColorTransferFunction *colorTransferFunction = vtkColorTransferFunction::New();
	colorTransferFunction->AddRGBPoint(0.0, 0.0, 0.0, 0.0);
	colorTransferFunction->AddRGBPoint(64.0, 1.0, 0.0, 0.0);
	colorTransferFunction->AddRGBPoint(128.0, 0.0, 0.0, 1.0);
	colorTransferFunction->AddRGBPoint( 192.0, 0.0, 1.0, 0.0);
	colorTransferFunction->AddRGBPoint( 255.0, 0.0, 0.2, 0.0);

	// The property describes how the data will look
	vtkVolumeProperty *volumeProperty = vtkVolumeProperty::New();
	volumeProperty->SetColor(colorTransferFunction);
	volumeProperty->SetScalarOpacity(opacityTransferFunction);

	// The mapper knows how to render the data
	vtkVolumeTextureMapper2D *volumeMapper = vtkVolumeTextureMapper2D::New();
	volumeMapper->SetInputConnection(reader->GetOutputPort());

	// The volume holds the mapper and the property and
	// can be used to position/orient the volume
	vtkVolume *volume = vtkVolume::New();
	volume->SetMapper(volumeMapper);
	volume->SetProperty(volumeProperty);

	ren1->AddVolume(volume);
	renWin->Render();


	iren->Initialize();



	iren->Start();



	return 0 ;
}