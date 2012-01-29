/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File: demo_vtk_Delaunary3D.cpp
	Purpose:
	    Ported from Delaunay3D.tcl

*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkDataSetMapper.h"
#include "vtkDelaunay3D.h"
#include "vtkMath.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkShrinkFilter.h"
#include "vtkProperty.h"
#include "vtkCamera.h"


// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	// This example shows how to use Delaunay3D with alpha shapes.
	//


	// The points to be triangulated are generated randomly in the unit cube
	// located at the origin. The points are then associated with a vtkPolyData.
	//
	vtkMath *math = vtkMath::New();
	vtkPoints *points = vtkPoints::New();
	const int NPOINTS = 500 ;
	for (int i=0;i<NPOINTS;i++)
		points->InsertNextPoint(vtkMath::Random(-1,1),vtkMath::Random(-1,1),vtkMath::Random(-1,1));


	vtkPolyData *profile = vtkPolyData::New();
	profile->SetPoints(points);

	// Delaunay3D is used to triangulate the points. The Tolerance is the distance
	// that nearly coincident points are merged together. (Delaunay does better if
	// points are well spaced.) The alpha value is the radius of circumcircles,
	// circumspheres. Any mesh entity whose circumcircle is smaller than this
	// value is output.
	//
	vtkDelaunay3D *del = vtkDelaunay3D::New();
	del->SetInput(profile);
	del->SetTolerance(0.01);
	del->SetAlpha(0.2);
	del->BoundingTriangulationOff();

	// Shrink the result to help see it better.
	vtkShrinkFilter *shrink = vtkShrinkFilter::New();
	shrink->SetInputConnection(del->GetOutputPort());
	shrink->SetShrinkFactor(0.9);

	vtkDataSetMapper *map = vtkDataSetMapper::New();
	map->SetInputConnection(shrink->GetOutputPort());

	vtkActor *triangulation = vtkActor::New();
	triangulation->SetMapper(map);
	triangulation->GetProperty()->SetColor(1,0, 0);
	triangulation->GetProperty()->BackfaceCullingOff();

	// Create graphics stuff
	//
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//
	ren1->AddActor(triangulation);

	ren1->SetBackground(1, 1, 1);
	renWin->SetSize(250, 250);
	renWin->Render();

	vtkCamera* cam1 = (ren1->GetActiveCamera());
	cam1->Zoom(1.5);

	// render the image
	//

      renWin->Render();






	iren->Initialize();
	iren->Start();


	return 0 ;
}