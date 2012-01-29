/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_medical_vol16_3.cpp
	Purpose:
		Ported from Medical3.tcl
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkCamera.h"
#include "vtkContourFilter.h"
#include "vtkImageActor.h"
#include "vtkImageMapToColors.h"
#include "vtkLookupTable.h"
#include "vtkOutlineFilter.h"
#include "vtkPolyDataMapper.h"
#include "vtkPolyDataNormals.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkStripper.h"
#include "vtkVolume16Reader.h"


#include "vtkProperty.h"
#include "vtkCamera.h"


// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	//
	// This example reads a volume dataset, extracts two isosurfaces that
	// represent the skin and bone, creates three orthogonal planes (saggital,
	// axial, coronal), and displays them.
	//

	// Create the renderer, the render window, and the interactor. The renderer
	// draws into the render window, the interactor enables mouse- and
	// keyboard-based interaction with the scene.
	//
	vtkRenderer *aRenderer = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(aRenderer);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// The following reader is used to read a series of 2D slices (images)
	// that compose the volume. The slice dimensions are set, and the
	// pixel spacing. The data Endianness must also be specified. The reader
	// usese the FilePrefix in combination with the slice number to construct
	// filenames using the format FilePrefix.%d. (In this case the FilePrefix
	// is the root name of the file: quarter.)
	vtkVolume16Reader *v16 = vtkVolume16Reader::New();
	v16->SetDataDimensions(64, 64);
	v16->SetDataByteOrderToLittleEndian();
	v16->SetFilePrefix(VTK_DATA_ROOT  "headsq\\quarter");
	v16->SetImageRange(1, 93);
	v16->SetDataSpacing( 3.2, 3.2, 1.5);

	// An isosurface, or contour value of 500 is known to correspond to the
	// skin of the patient. Once generated, a vtkPolyDataNormals filter is
	// is used to create normals for smooth surface shading during rendering.
	// The triangle stripper is used to create triangle strips from the
	// isosurface these render much faster on may systems.
	vtkContourFilter *skinExtractor = vtkContourFilter::New();
	skinExtractor->SetInputConnection(v16->GetOutputPort());
	skinExtractor->SetValue(0, 500);
	vtkPolyDataNormals *skinNormals = vtkPolyDataNormals::New();
	skinNormals->SetInputConnection(skinExtractor->GetOutputPort());
	skinNormals->SetFeatureAngle(60.0);
	vtkStripper *skinStripper = vtkStripper::New();
	skinStripper->SetInputConnection(skinNormals->GetOutputPort());
	vtkPolyDataMapper *skinMapper = vtkPolyDataMapper::New();
	skinMapper->SetInputConnection(skinStripper->GetOutputPort());
	skinMapper->ScalarVisibilityOff();
	vtkActor *skin = vtkActor::New();
	skin->SetMapper(skinMapper);
	skin->GetProperty()->SetDiffuseColor(1, .49, .25);
	skin->GetProperty()->SetSpecular(.3);
	skin->GetProperty()->SetSpecularPower(20);

	// An isosurface, or contour value of 1150 is known to correspond to the
	// skin of the patient. Once generated, a vtkPolyDataNormals filter is
	// is used to create normals for smooth surface shading during rendering.
	// The triangle stripper is used to create triangle strips from the
	// isosurface these render much faster on may systems.
	vtkContourFilter *boneExtractor = vtkContourFilter::New();
	boneExtractor->SetInputConnection(v16->GetOutputPort());
	boneExtractor->SetValue(0, 1150);
	vtkPolyDataNormals *boneNormals = vtkPolyDataNormals::New();
	boneNormals->SetInputConnection(boneExtractor->GetOutputPort());
	boneNormals->SetFeatureAngle(60.0);
	vtkStripper *boneStripper = vtkStripper::New();
	boneStripper->SetInputConnection(boneNormals->GetOutputPort());
	vtkPolyDataMapper *boneMapper = vtkPolyDataMapper::New();
	boneMapper->SetInputConnection(boneStripper->GetOutputPort());
	boneMapper->ScalarVisibilityOff();
	vtkActor *bone = vtkActor::New();
	bone->SetMapper(boneMapper);
	bone->GetProperty()->SetDiffuseColor( 1, 1, .9412);

	// An outline provides context around the data.
	//
	vtkOutlineFilter *outlineData = vtkOutlineFilter::New();
	outlineData->SetInputConnection(v16->GetOutputPort());
	vtkPolyDataMapper *mapOutline = vtkPolyDataMapper::New();
	mapOutline->SetInputConnection(outlineData->GetOutputPort());
	vtkActor *outline = vtkActor::New();
	outline->SetMapper(mapOutline);
	outline->GetProperty()->SetColor(0, 0, 0);

	// Now we are creating three orthogonal planes passing through the
	// volume. Each plane uses a different texture map and therefore has
	// diferent coloration.

	// Start by creatin a black/white lookup table.
	vtkLookupTable *bwLut = vtkLookupTable::New();
	bwLut->SetTableRange( 0, 2000);
	bwLut->SetSaturationRange( 0, 0);
	bwLut->SetHueRange( 0, 0);
	bwLut->SetValueRange( 0, 1);

	// Now create a lookup table that consists of the full hue circle (from HSV).
	vtkLookupTable *hueLut = vtkLookupTable::New();
	hueLut->SetTableRange( 0, 2000);
	hueLut->SetHueRange( 0, 1);
	hueLut->SetSaturationRange(1, 1);
	hueLut->SetValueRange(1, 1);

	// Finally, create a lookup table with a single hue but having a range
	// in the saturation of the hue.
	vtkLookupTable *satLut = vtkLookupTable::New();
	satLut->SetTableRange( 0, 2000);
	satLut->SetHueRange( .6, .6);
	satLut->SetSaturationRange( 0, 1);
	satLut->SetValueRange( 1, 1);

	// Create the first of the three planes. The filter vtkImageMapToColors
	// maps the data through the corresponding lookup table created above.
	// The vtkImageActor is a type of vtkProp and conveniently displays an image
	// on a single quadrilateral plane. It does this using texture mapping and
	// as a result is quite fast. (Note: the input image has to be unsigned
	// char values, which the vtkImageMapToColors produces.) Note also that
	// by specifying the DisplayExtent, the pipeline requests data of this
	// extent and the vtkImageMapToColors only processes a slice of data.
	vtkImageMapToColors *saggitalColors = vtkImageMapToColors::New();
	saggitalColors->SetInputConnection(v16->GetOutputPort());
	saggitalColors->SetLookupTable(bwLut);
	vtkImageActor *saggital = vtkImageActor::New();
	saggital->SetInput(saggitalColors->GetOutput());
	saggital->SetDisplayExtent(32, 32,  0, 63,  0, 92);

	// Create the second (axial) plane of the three planes. We use the same
	// approach as before except that the extent differs.
	vtkImageMapToColors *axialColors = vtkImageMapToColors::New();
	axialColors->SetInputConnection(v16->GetOutputPort());
	axialColors->SetLookupTable(hueLut);
	vtkImageActor *axial = vtkImageActor::New();
	axial->SetInput(axialColors->GetOutput());
	axial->SetDisplayExtent(0, 63 , 0, 63,  46, 46);

	// Create the third (coronal) plane of the three planes. We use the same
	// approach as before except that the extent differs.
	vtkImageMapToColors *coronalColors = vtkImageMapToColors::New();
	coronalColors->SetInputConnection(v16->GetOutputPort());
	coronalColors->SetLookupTable(satLut);
	vtkImageActor *coronal = vtkImageActor::New();
	coronal->SetInput(coronalColors->GetOutput());
	coronal->SetDisplayExtent(0, 63 , 32, 32, 0, 92);

	// It is convenient to create an initial view of the data. The FocalPoint
	// and Position form a vector direction. Later on (ResetCamera() method)
	// this vector is used to position the camera to look at the data in
	// this direction.
	vtkCamera *aCamera = vtkCamera::New();
	aCamera->SetViewUp( 0, 0, -1);
	aCamera->SetPosition( 0, 1, 0);
	aCamera->SetFocalPoint( 0, 0, 0);
	aCamera->ComputeViewPlaneNormal();

	// Actors are added to the renderer.
	aRenderer->AddActor(outline);
	aRenderer->AddActor(saggital);
	aRenderer->AddActor(axial);
	aRenderer->AddActor(coronal);
	aRenderer->AddActor(axial);
	aRenderer->AddActor(coronal);
	aRenderer->AddActor(skin);
	aRenderer->AddActor(bone);

	// Turn off bone for this example.
	bone->VisibilityOff();

	// Set skin to semi-transparent.
	skin->GetProperty()->SetOpacity(0.5);

	// An initial camera view is created.  The Dolly() method moves
	// the camera towards the FocalPoint, thereby enlarging the image.
	aRenderer->SetActiveCamera(aCamera);
	aRenderer->ResetCamera();
	aCamera->Dolly(1.5);

	// Set a background color for the renderer and set the size of the
	// render window (expressed in pixels).
	aRenderer->SetBackground(1, 1, 1);
	renWin->SetSize(640, 480);

	// Note that when camera movement occurs (as it does in the Dolly()
	// method), the clipping planes often need adjusting. Clipping planes
	// consist of two planes: near and far along the view direction. The
	// near plane clips out objects in front of the plane the far plane
	// clips out objects behind the plane. This way only what is drawn
	// between the planes is actually rendered.
	aRenderer->ResetCameraClippingRange();


	iren->Initialize();
	iren->Start();


	return 0 ;
}