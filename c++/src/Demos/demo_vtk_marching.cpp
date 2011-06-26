/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	  demo_vtk_marching.cpp
	Purpose:
	    Ported from marching.tcl

*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkContourFilter.h"
#include "vtkCubeSource.h"
#include "vtkExtractEdges.h"
#include "vtkFloatArray.h"
#include "vtkGlyph3D.h"
#include "vtkIdList.h"
#include "vtkPoints.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkShrinkPolyData.h"
#include "vtkSphereSource.h"
#include "vtkThresholdPoints.h"
#include "vtkTransform.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkTubeFilter.h"
#include "vtkUnstructuredGrid.h"
#include "vtkVectorText.h"
#include "vtkPointData.h"

#include "vtkProperty.h"
#include "vtkCamera.h"

// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	//define a Single Cube
	vtkFloatArray *Scalars = vtkFloatArray::New();
		Scalars->InsertNextValue(1.0);
		Scalars->InsertNextValue(0.0);
		Scalars->InsertNextValue(0.0);
		Scalars->InsertNextValue(1.0);
		Scalars->InsertNextValue(0.0);
		Scalars->InsertNextValue(0.0);
		Scalars->InsertNextValue(0.0);
		Scalars->InsertNextValue(0.0);

	vtkPoints *Points = vtkPoints::New();
	Points->InsertNextPoint(0, 0, 0);
	Points->InsertNextPoint(1, 0, 0);
	Points->InsertNextPoint(1, 1, 0);
	Points->InsertNextPoint(0, 1, 0);
	Points->InsertNextPoint(0, 0, 1);
	Points->InsertNextPoint(1, 0, 1);
	Points->InsertNextPoint(1, 1, 1);
	Points->InsertNextPoint(0, 1, 1);

	vtkIdList *Ids = vtkIdList::New();
		Ids->InsertNextId(0);
		Ids->InsertNextId(1);
		Ids->InsertNextId(2);
		Ids->InsertNextId(3);
		Ids->InsertNextId(4);
		Ids->InsertNextId(5);
		Ids->InsertNextId(6);
		Ids->InsertNextId(7);

	vtkUnstructuredGrid *Grid = vtkUnstructuredGrid::New();
	Grid->Allocate(10, 10);
		Grid->InsertNextCell(12, Ids);
	Grid->SetPoints(Points);
	Grid->GetPointData()->SetScalars(Scalars);

	// Find the triangles that lie along the 0.5 contour in this cube.
	vtkContourFilter *Marching = vtkContourFilter::New();
		Marching->SetInput(Grid);
	Marching->SetValue(0, 0.5);
		Marching->Update();

	// Extract the edges of the triangles just found.
	vtkExtractEdges *triangleEdges = vtkExtractEdges::New();
		triangleEdges->SetInputConnection(Marching->GetOutputPort());
	// Draw the edges as tubes instead of lines.  Also create the associated
	// mapper and actor to display the tubes.
	vtkTubeFilter *triangleEdgeTubes = vtkTubeFilter::New();
		triangleEdgeTubes->SetInputConnection(triangleEdges->GetOutputPort());
		triangleEdgeTubes->SetRadius(.005);
		triangleEdgeTubes->SetNumberOfSides(6);
		triangleEdgeTubes->UseDefaultNormalOn();
		triangleEdgeTubes->SetDefaultNormal(.577, .577, .577);
	vtkPolyDataMapper *triangleEdgeMapper = vtkPolyDataMapper::New();
		triangleEdgeMapper->SetInputConnection(triangleEdgeTubes->GetOutputPort());
		triangleEdgeMapper->ScalarVisibilityOff();
	vtkActor *triangleEdgeActor = vtkActor::New();
		triangleEdgeActor->SetMapper(triangleEdgeMapper);
		triangleEdgeActor->GetProperty()->SetDiffuseColor(0.3,0.3,0.3);
		triangleEdgeActor->GetProperty()->SetSpecular(.4);
		triangleEdgeActor->GetProperty()->SetSpecularPower(10);

	// Shrink the triangles we found earlier.  Create the associated mapper
	// and actor.  Set the opacity of the shrunken triangles.
	vtkShrinkPolyData *aShrinker = vtkShrinkPolyData::New();
		aShrinker->SetShrinkFactor(1);
		aShrinker->SetInputConnection(Marching->GetOutputPort());
	vtkPolyDataMapper *aMapper = vtkPolyDataMapper::New();
		aMapper->ScalarVisibilityOff();
		aMapper->SetInputConnection(aShrinker->GetOutputPort());
	vtkActor *Triangles = vtkActor::New();
		Triangles->SetMapper(aMapper);
		Triangles->GetProperty()->SetDiffuseColor(0,1,1);
		Triangles->GetProperty()->SetOpacity(.6);

	// Draw a cube the same size and at the same position as the one created
	// previously.  Extract the edges because we only want to see the outline
	// of the cube.  Pass the edges through a vtkTubeFilter so they are displayed
	// as tubes rather than lines.
	vtkCubeSource *CubeModel = vtkCubeSource::New();
		CubeModel->SetCenter(.5, .5, .5);
	vtkExtractEdges *Edges = vtkExtractEdges::New();
		Edges->SetInputConnection(CubeModel->GetOutputPort());
	vtkTubeFilter *Tubes = vtkTubeFilter::New();
		Tubes->SetInputConnection(Edges->GetOutputPort());
		Tubes->SetRadius(.01);
		Tubes->SetNumberOfSides(6);
		Tubes->UseDefaultNormalOn();
		Tubes->SetDefaultNormal(.577, .577, .577);
	// Create the mapper and actor to display the cube edges.
	vtkPolyDataMapper *TubeMapper = vtkPolyDataMapper::New();
		TubeMapper->SetInputConnection(Tubes->GetOutputPort());
	vtkActor *CubeEdges = vtkActor::New();
		CubeEdges->SetMapper(TubeMapper);
		CubeEdges->GetProperty()->SetDiffuseColor(1,0,0);
		CubeEdges->GetProperty()->SetSpecular(.4);
		CubeEdges->GetProperty()->SetSpecularPower(10);

	// Create a sphere to use as a glyph source for vtkGlyph3D.
	vtkSphereSource *Sphere = vtkSphereSource::New();
	Sphere->SetRadius(0.04);
	Sphere->SetPhiResolution(20);
	Sphere->SetThetaResolution(20);
	// Remove the part of the cube with data values below 0.5.
	vtkThresholdPoints *ThresholdIn = vtkThresholdPoints::New();
	ThresholdIn->SetInput(Grid);
	ThresholdIn->ThresholdByUpper(.5);
	// Display spheres at the vertices remaining in the cube data set after
	// it was passed through vtkThresholdPoints.
	vtkGlyph3D *Vertices = vtkGlyph3D::New();
	Vertices->SetInputConnection(ThresholdIn->GetOutputPort());
	Vertices->SetSource(Sphere->GetOutput());
	// Create a mapper and actor to display the glyphs.
	vtkPolyDataMapper *SphereMapper = vtkPolyDataMapper::New();
	SphereMapper->SetInputConnection(Vertices->GetOutputPort());
	SphereMapper->ScalarVisibilityOff();
	vtkActor *CubeVertices = vtkActor::New();
	CubeVertices->SetMapper(SphereMapper);
	CubeVertices->GetProperty()->SetDiffuseColor(0.8, 0.3,0.3);


	// Define the text for the label
	vtkVectorText *caseLabel = vtkVectorText::New();
	caseLabel->SetText("Case 1");

	// Set up a transform to move the label to a new position.
	vtkTransform *aLabelTransform = vtkTransform::New();
	aLabelTransform->Identity();
	aLabelTransform->Translate( -0.2, 0, 1.25);
	aLabelTransform->Scale(.05, .05, .05);

	// Move the label to a new position.
	vtkTransformPolyDataFilter *labelTransform = vtkTransformPolyDataFilter::New();
	labelTransform->SetTransform(aLabelTransform);
	labelTransform->SetInputConnection(caseLabel->GetOutputPort());
	;
	// Create a mapper and actor to display the text.
	vtkPolyDataMapper *labelMapper = vtkPolyDataMapper::New();
	labelMapper->SetInputConnection(labelTransform->GetOutputPort());
	;
	vtkActor *labelActor = vtkActor::New();
	labelActor->SetMapper(labelMapper);
	;
	// Define the base that the cube sits on.  Create its associated mapper
	// and actor.  Set the position of the actor.
	vtkCubeSource *baseModel = vtkCubeSource::New();
	baseModel->SetXLength(1.5);
	baseModel->SetYLength(.01);
	baseModel->SetZLength(1.5);
	vtkPolyDataMapper *baseMapper = vtkPolyDataMapper::New();
	baseMapper->SetInputConnection(baseModel->GetOutputPort());
	vtkActor *base = vtkActor::New();
	base->SetMapper(baseMapper);
	base->SetPosition(.5, -.09, .5);

	// Create the Renderer, RenderWindow, and RenderWindowInteractor
	//
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	renWin->SetSize(640, 480);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer
	ren1->AddActor(triangleEdgeActor);
	ren1->AddActor(base);
	ren1->AddActor(labelActor);
	ren1->AddActor(CubeEdges);
	ren1->AddActor(CubeVertices);
	ren1->AddActor(Triangles);

	// Set the background color.
	ren1->SetBackground(0.9,0.9,0.9);

	// Set the scalar values for this case of marching cubes.
	// case12->Scalars(0, 1);
	// Force the grid to update.
	Grid->Modified();

	// Position the camera.
	ren1->ResetCamera();
	ren1->GetActiveCamera()->Dolly(1.2);
	ren1->GetActiveCamera()->Azimuth(30);
	ren1->GetActiveCamera()->Elevation(20);
	ren1->ResetCameraClippingRange();

	// Render
	renWin->Render();


	iren->Initialize();
	iren->Start();



	return 0 ;
}