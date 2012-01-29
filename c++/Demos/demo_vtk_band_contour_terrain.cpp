/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_band_contour_terrain.cpp
	Purpose:
		Ported from BandCoutourTerrain.tcl
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkBandedPolyDataContourFilter.h"
#include "vtkCamera.h"
#include "vtkDEMReader.h"
#include "vtkImageDataGeometryFilter.h"
#include "vtkImageShrink3D.h"
#include "vtkLODActor.h"
#include "vtkLookupTable.h"
#include "vtkPolyDataMapper.h"
#include "vtkPolyDataNormals.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkWarpScalar.h"
#include "vtkImageData.h"

#include "vtkProperty.h"
#include "vtkCamera.h"


// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	// In this example we show the use of the vtkBandedPolyDataContourFilter.
	// This filter creates separate, constant colored bands for a range of scalar
	// values. Each band is bounded by two scalar values, and the cell data lying
	// within the value has the same cell scalar value.


	// The lookup table is similar to that used by maps. Two hues are used: a
	// brown for land, and a blue for water. The value of the hue is changed to
	// give the effect of elevation.
	int Scale = (5);
	vtkLookupTable *lutWater = vtkLookupTable::New();
	lutWater->SetNumberOfColors(10);
	lutWater->SetHueRange(0.58, 0.58);
	lutWater->SetSaturationRange(0.5, 0.1);
	lutWater->SetValueRange(0.5, 1.0);
	lutWater->Build();
	vtkLookupTable *lutLand = vtkLookupTable::New();
	lutLand->SetNumberOfColors(10);
	lutLand->SetHueRange(0.1, 0.1);
	lutLand->SetSaturationRange(0.4, 0.1);
	lutLand->SetValueRange(0.55, 0.9);
	lutLand->Build();


	// The DEM reader reads data and creates an output image.
	vtkDEMReader *demModel = vtkDEMReader::New();
	demModel->SetFileName(VTK_DATA_ROOT "SainteHelens.dem" );
	demModel->Update();

	// We shrink the terrain data down a bit to yield better performance for
	// this example.
	int shrinkFactor = (4);
	vtkImageShrink3D *shrink = vtkImageShrink3D::New();
	shrink->SetShrinkFactors(shrinkFactor, shrinkFactor, 1);
	shrink->SetInputConnection(demModel->GetOutputPort());
	shrink->AveragingOn();

	// Convert the image into polygons.
	vtkImageDataGeometryFilter *geom = vtkImageDataGeometryFilter::New();
	geom->SetInputConnection(shrink->GetOutputPort());

	// Warp the polygons based on elevation.
	vtkWarpScalar *warp = vtkWarpScalar::New();
	warp->SetInputConnection(geom->GetOutputPort());
	warp->SetNormal(0, 0, 1);
	warp->UseNormalOn();
	warp->SetScaleFactor(Scale);

	// Create the contour bands.
	vtkBandedPolyDataContourFilter *bcf = vtkBandedPolyDataContourFilter::New();
	bcf->SetInput(warp->GetPolyDataOutput());
	bcf->GenerateValues(15, demModel->GetOutput()->GetScalarRange());
	bcf->SetScalarModeToIndex();
	bcf->GenerateContourEdgesOn();

	// Compute normals to give a better look.
	vtkPolyDataNormals *normals = vtkPolyDataNormals::New();
	normals->SetInputConnection(bcf->GetOutputPort());
	normals->SetFeatureAngle(60);
	normals->ConsistencyOff();
	normals->SplittingOff();

	vtkPolyDataMapper *demMapper = vtkPolyDataMapper::New();
	demMapper->SetInputConnection(normals->GetOutputPort());
	demMapper->SetScalarRange( 0, 10);
	demMapper->SetLookupTable(lutLand);
	demMapper->SetScalarModeToUseCellData();

	vtkLODActor *demActor = vtkLODActor::New();
	demActor->SetMapper(demMapper);

	//# Create contour edges
	vtkPolyDataMapper *edgeMapper = vtkPolyDataMapper::New();
	edgeMapper->SetInput(bcf->GetContourEdgesOutput());
	edgeMapper->SetResolveCoincidentTopologyToPolygonOffset();
	vtkActor *edgeActor = vtkActor::New();
	edgeActor->SetMapper(edgeMapper);
	edgeActor->GetProperty()->SetColor(0, 0, 0);

	//# Test clipping
	// Create the contour bands.
	vtkBandedPolyDataContourFilter *bcf2 = vtkBandedPolyDataContourFilter::New();
	bcf2->SetInput(warp->GetPolyDataOutput());
	bcf2->ClippingOn();
	bcf2->GenerateValues( 10, 1000, 2000);
	bcf2->SetScalarModeToValue();

	// Compute normals to give a better look.
	vtkPolyDataNormals *normals2 = vtkPolyDataNormals::New();
	normals2->SetInputConnection(bcf2->GetOutputPort());
	normals2->SetFeatureAngle(60);
	normals2->ConsistencyOff();
	normals2->SplittingOff();

	vtkLookupTable *lut = vtkLookupTable::New();
	lut->SetNumberOfColors(10);
	vtkPolyDataMapper *demMapper2 = vtkPolyDataMapper::New();
	demMapper2->SetInputConnection(normals2->GetOutputPort());
	demMapper2->SetScalarRange( demModel->GetOutput()->GetScalarRange());
	demMapper2->SetLookupTable(lut);
	demMapper2->SetScalarModeToUseCellData();

	vtkLODActor *demActor2 = vtkLODActor::New();
	demActor2->SetMapper(demMapper2);
	demActor2->AddPosition(0, 15000, 0);

	// Create the RenderWindow, Renderer and both Actors
	//
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//
	ren1->AddActor(demActor);
	ren1->AddActor(demActor2);
	ren1->AddActor(edgeActor);

	ren1->SetBackground(.4, .4, .4);
	renWin->SetSize(375, 200);

	vtkCamera *cam = vtkCamera::New();
	cam->SetPosition(-17438.8, 2410.62, 25470.8);
	cam->SetFocalPoint(3985.35, 11930.6, 5922.14);
	cam->SetViewUp(0, 0, 1);
	ren1->SetActiveCamera(cam);
	ren1->ResetCamera();
	cam->Zoom(2);
	iren->SetDesiredUpdateRate(1);


	renWin->Render();

	iren->Initialize();
	iren->Start();



	return 0 ;
}