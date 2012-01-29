/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
 	File: demo_vtk_delMesh.cpp
	Purpose:
        Ported from DelMesh.tcl
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkDelaunay2D.h"
#include "vtkExtractEdges.h"
#include "vtkGlyph3D.h"
#include "vtkMath.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSphereSource.h"
#include "vtkTubeFilter.h"
#include "vtkProperty.h"
#include "vtkCamera.h"


// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	// Generate some random points
	//

	vtkPoints *points = vtkPoints::New();
	const int NPoints = 50 ;
	for (int i=0;i<NPoints;i++)
		points->InsertNextPoint(vtkMath::Random(0,1),vtkMath::Random(0,1),0);


	// Create a polydata with the points we just created.
	vtkPolyData *profile = vtkPolyData::New();
	profile->SetPoints(points);

	// Perform a 2D Delaunay triangulation on them.
	//
	vtkDelaunay2D *del = vtkDelaunay2D::New();
	del->SetInput(profile);
	del->SetTolerance(0.001);
	vtkPolyDataMapper *mapMesh = vtkPolyDataMapper::New();
	mapMesh->SetInputConnection(del->GetOutputPort());
	vtkActor *meshActor = vtkActor::New();
	meshActor->SetMapper(mapMesh);
	meshActor->GetProperty()->SetColor(.1, .2, .4);

	// We will now create a nice looking mesh by wrapping the edges in tubes,
	// and putting fat spheres at the points.
	vtkExtractEdges *extract = vtkExtractEdges::New();
	extract->SetInputConnection(del->GetOutputPort());
	vtkTubeFilter *tubes = vtkTubeFilter::New();
	tubes->SetInputConnection(extract->GetOutputPort());
	tubes->SetRadius(0.01);
	tubes->SetNumberOfSides(6);
	vtkPolyDataMapper *mapEdges = vtkPolyDataMapper::New();
	mapEdges->SetInputConnection(tubes->GetOutputPort());
	vtkActor *edgeActor = vtkActor::New();
	edgeActor->SetMapper(mapEdges);
	edgeActor->GetProperty()->SetColor(1,0,0);
	edgeActor->GetProperty()->SetSpecularColor(1,1, 1);
	edgeActor->GetProperty()->SetSpecular(0.3);
	edgeActor->GetProperty()->SetSpecularPower(20);
	edgeActor->GetProperty()->SetAmbient(0.2);
	edgeActor->GetProperty()->SetDiffuse(0.8);

	vtkSphereSource *ball = vtkSphereSource::New();
	ball->SetRadius(0.025);
	ball->SetThetaResolution(12);
	ball->SetPhiResolution(12);
	vtkGlyph3D *balls = vtkGlyph3D::New();
	balls->SetInputConnection(del->GetOutputPort());
	balls->SetSourceConnection(ball->GetOutputPort());
	vtkPolyDataMapper *mapBalls = vtkPolyDataMapper::New();
	mapBalls->SetInputConnection(balls->GetOutputPort());
	vtkActor *ballActor = vtkActor::New();
	ballActor->SetMapper(mapBalls);
	ballActor->GetProperty()->SetColor(0,1,0);
	ballActor->GetProperty()->SetSpecularColor(1,1, 1);
	ballActor->GetProperty()->SetSpecular(0.3);
	ballActor->GetProperty()->SetSpecularPower(20);
	ballActor->GetProperty()->SetAmbient(0.2);
	ballActor->GetProperty()->SetDiffuse(0.8);

	// Create graphics objects
	// Create the rendering window, renderer, and interactive renderer
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	ren1->AddActor(ballActor);
	ren1->AddActor(edgeActor);
	ren1->SetBackground(1, 1, 1);
	renWin->SetSize(640, 480);

	// render the image
	//

	ren1->ResetCamera();
	ren1->GetActiveCamera()->Zoom(1.5);
	iren->Initialize();
	iren->Start();

	return 0 ;
}