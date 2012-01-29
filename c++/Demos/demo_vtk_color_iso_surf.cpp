/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_color_iso_surf.cpp
	Purpose:
		Ported from Colorlsosurface.tcl
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkContourFilter.h"
#include "vtkLODActor.h"
#include "vtkPLOT3DReader.h"
#include "vtkPolyDataMapper.h"
#include "vtkPolyDataNormals.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkStructuredGridOutlineFilter.h"


#include "vtkProperty.h"
#include "vtkCamera.h"


// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	// This example shows how to color an isosurface with other data. Basically
	// an isosurface is generated, and a data array is selected and used by the
	// mapper to color the surface.


	// Read some data. The important thing here is to read a function as a data
	// array as well as the scalar and vector.  (here function 153 is named
	// "Velocity Magnitude").Later this data array will be used to color the
	// isosurface.
	//
	vtkPLOT3DReader *pl3d = vtkPLOT3DReader::New();
	pl3d->SetXYZFileName(VTK_DATA_ROOT "combxyz.bin");
	pl3d->SetQFileName(VTK_DATA_ROOT "combq.bin");
	pl3d->SetScalarFunctionNumber(100);
	pl3d->SetVectorFunctionNumber(202);
	pl3d->AddFunction(153);
	pl3d->Update();
	pl3d->DebugOn();

	// The contoru filter uses the labeled scalar (function number 100
	// above to generate the contour surface; all other data is interpolated
	// during the contouring process.
	//
	vtkContourFilter *iso = vtkContourFilter::New();
	iso->SetInputConnection(pl3d->GetOutputPort());
	iso->SetValue(0, .24);

	vtkPolyDataNormals *normals = vtkPolyDataNormals::New();
	normals->SetInputConnection(iso->GetOutputPort());
	normals->SetFeatureAngle(45);

	// We indicate to the mapper to use the velcoity magnitude, which is a
	// vtkDataArray that makes up part of the point attribute data.
	//
	vtkPolyDataMapper *isoMapper = vtkPolyDataMapper::New();
	isoMapper->SetInputConnection(normals->GetOutputPort());
	isoMapper->ScalarVisibilityOn();
	isoMapper->SetScalarRange(0, 1500);
	isoMapper->SetScalarModeToUsePointFieldData();
	isoMapper->ColorByArrayComponent("VelocityMagnitude", 0);

	vtkLODActor *isoActor = vtkLODActor::New();
	isoActor->SetMapper(isoMapper);
	isoActor->SetNumberOfCloudPoints(1000);

	vtkStructuredGridOutlineFilter *outline = vtkStructuredGridOutlineFilter::New();
	outline->SetInputConnection(pl3d->GetOutputPort());
	vtkPolyDataMapper *outlineMapper = vtkPolyDataMapper::New();
	outlineMapper->SetInputConnection(outline->GetOutputPort());
	vtkActor *outlineActor = vtkActor::New();
	outlineActor->SetMapper(outlineMapper);

	// Create the usual rendering stuff.
	//
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//
	ren1->AddActor(outlineActor);
	ren1->AddActor(isoActor);
	ren1->SetBackground(1, 1, 1);
	renWin->SetSize(500, 500);
	ren1->SetBackground(0.1, 0.2, 0.4);

	vtkCamera* cam1 = (ren1->GetActiveCamera());
	cam1->SetClippingRange(3.95297, 50);
	cam1->SetFocalPoint(9.71821, 0.458166, 29.3999);
	cam1->SetPosition(2.7439, -37.3196, 38.7167);
	cam1->SetViewUp(-0.16123, 0.264271, 0.950876);

	// render the image
	//

	renWin->Render();

	iren->Initialize();
	iren->Start();



	return 0 ;
}