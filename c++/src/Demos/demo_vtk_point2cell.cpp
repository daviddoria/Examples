/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_point2cell.cpp
	Purpose:
		Ported from pointToCellData.tcl
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkConnectivityFilter.h"
#include "vtkContourFilter.h"
#include "vtkDataSetMapper.h"
#include "vtkGeometryFilter.h"
#include "vtkLookupTable.h"
#include "vtkPointDataToCellData.h"
#include "vtkPolyDataMapper.h"
#include "vtkPolyDataNormals.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkThreshold.h"
#include "vtkUnstructuredGridReader.h"
#include "vtkWarpVector.h"

#include "vtkProperty.h"
#include "vtkCamera.h"

// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	// This example demonstrates the conversion of point data to cell data.
	// The conversion is necessary because we want to threshold data based
	// on cell scalar values.


	// Read some data with point data attributes. The data is from a plastic
	// blow molding process (e.g., to make plastic bottles) and consists of two
	// logical components: a mold and a parison. The parison is the
	// hot plastic that is being molded, and the mold is clamped around the
	// parison to form its shape.
	vtkUnstructuredGridReader *reader = vtkUnstructuredGridReader::New();
	reader->SetFileName( VTK_DATA_ROOT "blow.vtk");
	reader->SetScalarsName("thickness9");
	reader->SetVectorsName("displacement9");

	// Convert the point data to cell data. The point data is passed through the
	// filter so it can be warped. The vtkThresholdFilter then thresholds based
	// on cell scalar values and extracts a portion of the parison whose cell
	// scalar values lie between 0.25 and 0.75.
	vtkPointDataToCellData *p2c = vtkPointDataToCellData::New();
	p2c->SetInputConnection(reader->GetOutputPort());
	p2c->PassPointDataOn();
	vtkWarpVector *warp = vtkWarpVector::New();
	warp->SetInputConnection(p2c->GetOutputPort());
	vtkThreshold *thresh = vtkThreshold::New();
	thresh->SetInputConnection(warp->GetOutputPort());
	thresh->ThresholdBetween(0.25, 0.75);
	thresh->SetInputArrayToProcess(1, 0, 0, 0, "thickness9");
	//    thresh SetAttributeModeToUseCellData

	// This is used to extract the mold from the parison.
	vtkConnectivityFilter *connect = vtkConnectivityFilter::New();
	connect->SetInputConnection(thresh->GetOutputPort());
	connect->SetExtractionModeToSpecifiedRegions();
	connect->AddSpecifiedRegion(0);
	connect->AddSpecifiedRegion(1);
	vtkDataSetMapper *moldMapper = vtkDataSetMapper::New();
	moldMapper->SetInputConnection(reader->GetOutputPort());
	moldMapper->ScalarVisibilityOff();
	vtkActor *moldActor = vtkActor::New();
	moldActor->SetMapper(moldMapper);
	moldActor->GetProperty()->SetColor(.2, .2, .2);
	;

	// The threshold filter has been used to extract the parison.
	vtkConnectivityFilter *connect2 = vtkConnectivityFilter::New();
	connect2->SetInputConnection(thresh->GetOutputPort());
	vtkGeometryFilter *parison = vtkGeometryFilter::New();
	parison->SetInputConnection(connect2->GetOutputPort());
	vtkPolyDataNormals *normals2 = vtkPolyDataNormals::New();
	normals2->SetInputConnection(parison->GetOutputPort());
	normals2->SetFeatureAngle(60);
	vtkLookupTable *lut = vtkLookupTable::New();
	lut->SetHueRange(0.0, 0.66667);
	vtkPolyDataMapper *parisonMapper = vtkPolyDataMapper::New();
	parisonMapper->SetInputConnection(normals2->GetOutputPort());
	parisonMapper->SetLookupTable(lut);
	parisonMapper->SetScalarRange(0.12, 1.0);
	vtkActor *parisonActor = vtkActor::New();
	parisonActor->SetMapper(parisonMapper);

	// We generate some contour lines on the parison.
	vtkContourFilter *cf = vtkContourFilter::New();
	cf->SetInputConnection(connect2->GetOutputPort());
	cf->SetValue(0, .5);
	vtkPolyDataMapper *contourMapper = vtkPolyDataMapper::New();
	contourMapper->SetInputConnection(cf->GetOutputPort());
	vtkActor *contours = vtkActor::New();
	contours->SetMapper(contourMapper);

	// Create graphics stuff
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	ren1->AddActor(moldActor);
	ren1->AddActor(parisonActor);
	ren1->AddActor(contours);

	ren1->ResetCamera();
	ren1->GetActiveCamera()->Azimuth(60);
	ren1->GetActiveCamera()->Roll(-90);
	ren1->GetActiveCamera()->Dolly(2);
	ren1->ResetCameraClippingRange();

	ren1->SetBackground(1, 1, 1);
	renWin->SetSize(750, 400);

	iren->Initialize();


	iren->Start();



	return 0 ;
}