/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	Purpose:

*/
#include "project_config.h"

#include "vtkActor.h" 
#include "vtkAppendPolyData.h" 
#include "vtkPLOT3DReader.h" 
#include "vtkPolyDataMapper.h" 
#include "vtkPolyDataNormals.h" 
#include "vtkRenderWindow.h" 
#include "vtkRenderWindowInteractor.h" 
#include "vtkRenderer.h" 
#include "vtkStructuredGridGeometryFilter.h" 
#include "vtkStructuredGridOutlineFilter.h" 
#include "vtkWarpScalar.h" 
#include "vtkStructuredGrid.h"
#include "vtkProperty.h"
#include "vtkCamera.h"

// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"

int main(int argc, char** argv)
{
	// Here we read data from a annular combustor. A combustor burns fuel and air
	// in a gas turbine (e.g., a jet engine) and the hot gas eventually makes its
	// way to the turbine section.
	//
	vtkPLOT3DReader *pl3d = vtkPLOT3DReader::New();
	pl3d->SetXYZFileName(VTK_DATA_ROOT "combxyz.bin");
	pl3d->SetQFileName(VTK_DATA_ROOT "combq.bin");
	pl3d->SetScalarFunctionNumber(100);
	pl3d->SetVectorFunctionNumber(202);
	pl3d->Update();

	// Planes are specified using a imin,imax, jmin,jmax, kmin,kmax coordinate
	// specification. Min and max i,j,k values are clamped to 0 and maximum value.
	//
	vtkStructuredGridGeometryFilter *plane = vtkStructuredGridGeometryFilter::New();
	plane->SetInputConnection(pl3d->GetOutputPort());
	plane->SetExtent(10, 10, 1, 100, 1, 100);
	vtkStructuredGridGeometryFilter *plane2 = vtkStructuredGridGeometryFilter::New();
	plane2->SetInputConnection(pl3d->GetOutputPort());
	plane2->SetExtent(30, 30, 1, 100, 1, 100);
	vtkStructuredGridGeometryFilter *plane3 = vtkStructuredGridGeometryFilter::New();
	plane3->SetInputConnection(pl3d->GetOutputPort());
	plane3->SetExtent(45, 45, 1, 100, 1, 100);

	// We use an append filter because that way we can do the warping, etc. just
	// using a single pipeline and actor.
	//
	vtkAppendPolyData *appendF = vtkAppendPolyData::New();
	appendF->AddInputConnection(plane->GetOutputPort());
	appendF->AddInputConnection(plane2->GetOutputPort());
	appendF->AddInputConnection(plane3->GetOutputPort());
	vtkWarpScalar *warp = vtkWarpScalar::New();
	warp->SetInputConnection(appendF->GetOutputPort());
	warp->UseNormalOn();
	warp->SetNormal(1.0, 0.0, 0.0);
	warp->SetScaleFactor(2.5);
	vtkPolyDataNormals *normals = vtkPolyDataNormals::New();
	normals->SetInputConnection(warp->GetOutputPort());
	normals->SetFeatureAngle(60);
	vtkPolyDataMapper *planeMapper = vtkPolyDataMapper::New();
	planeMapper->SetInputConnection(normals->GetOutputPort());
	planeMapper->SetScalarRange(pl3d->GetOutput()->GetScalarRange());
	vtkActor *planeActor = vtkActor::New();
	planeActor->SetMapper(planeMapper);

	// The outline provides context for the data and the planes.
	vtkStructuredGridOutlineFilter *outline = vtkStructuredGridOutlineFilter::New();
	outline->SetInputConnection(pl3d->GetOutputPort());
	vtkPolyDataMapper *outlineMapper = vtkPolyDataMapper::New();
	outlineMapper->SetInputConnection(outline->GetOutputPort());
	vtkActor *outlineActor = vtkActor::New();
	outlineActor->SetMapper(outlineMapper);
	outlineActor->GetProperty()->SetColor(0, 0, 0);

	// Create the usual graphics stuff/
	//
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	ren1->AddActor(outlineActor);
	ren1->AddActor(planeActor);
	ren1->SetBackground(1, 1, 1);
	renWin->SetSize(500, 500);

	// Create an initial view.
	vtkCamera* cam1 = (ren1->GetActiveCamera());
	cam1->SetClippingRange(3.95297, 50);
	cam1->SetFocalPoint(8.88908, 0.595038, 29.3342);
	cam1->SetPosition(-12.3332, 31.7479, 41.2387);
	cam1->SetViewUp(0.060772, -0.319905, 0.945498);
	iren->Initialize();
	iren->Start();


	return 0 ;
}