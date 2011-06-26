/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_cspline.cpp
	Purpose:
		Ported from CSpline.tcl
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkCardinalSpline.h"
#include "vtkCellArray.h"
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
	// This example demonstrates the use of vtkCardinalSpline.
	// It creates random points and connects them with a spline

	//
	// First we include the VTK Tcl packages which will make available
	// all of the vtk commands to Tcl
	//


	// This will be used later to get random numbers.
	vtkMath *math = vtkMath::New();

	// Total number of points.
	int numberOfInputPoints = (10);

	// One spline for each direction.
	vtkCardinalSpline *aSplineX = vtkCardinalSpline::New();
	vtkCardinalSpline *aSplineY = vtkCardinalSpline::New();
	vtkCardinalSpline *aSplineZ = vtkCardinalSpline::New();

	// Generate random (pivot) points and add the corresponding
	// coordinates to the splines.
	// aSplineX will interpolate the x values of the points
	// aSplineY will interpolate the y values of the points
	// aSplineZ will interpolate the z values of the points
	vtkPoints *inputPoints = vtkPoints::New();
	for (int i=0;i<numberOfInputPoints;i++)
	{
		double x = math->Random(0,1);
		double y = math->Random(0,1);
		double z = math->Random(0,1);
		aSplineX->AddPoint(i, x);
		aSplineY->AddPoint(i, y);
		aSplineZ->AddPoint(i, z);
		inputPoints->InsertNextPoint(x, y, z);
	}


	// The following section will create glyphs for the pivot points
	// in order to make the effect of the spline more clear.

	// Create a polydata to be glyphed.
	vtkPolyData *inputData = vtkPolyData::New();
	inputData->SetPoints(inputPoints);

	// Use sphere as glyph source.
	vtkSphereSource *balls = vtkSphereSource::New();
	balls->SetRadius(.01);
	balls->SetPhiResolution(10);
	balls->SetThetaResolution(10);

	vtkGlyph3D *glyphPoints = vtkGlyph3D::New();
	glyphPoints->SetInput(inputData);
	glyphPoints->SetSource(balls->GetOutput());

	vtkPolyDataMapper *glyphMapper = vtkPolyDataMapper::New();
	glyphMapper->SetInputConnection(glyphPoints->GetOutputPort());

	vtkActor *glyph = vtkActor::New();
	glyph->SetMapper(glyphMapper);
	glyph->GetProperty()->SetDiffuseColor(0.8,0.0,0.0);
	glyph->GetProperty()->SetSpecular(.3);
	glyph->GetProperty()->SetSpecularPower(30);

	// Generate the polyline for the spline.
	vtkPoints *points = vtkPoints::New();
	vtkPolyData *profileData = vtkPolyData::New();

	// Number of points on the spline
	int numberOfOutputPoints = (400);

	// Interpolate x, y and z by using the three spline filters and
	// create new points
	for (int i=0;i<numberOfOutputPoints;i++)
	{
		double t = (numberOfInputPoints-1.0)/(numberOfOutputPoints-1.0)*i ;
		points->InsertPoint(i, aSplineX->Evaluate(t), aSplineY->Evaluate(t), aSplineZ->Evaluate(t) );

	}


	// Create the polyline.
	vtkCellArray *lines = vtkCellArray::New();
	lines->InsertNextCell(numberOfOutputPoints);
	for (int i=0;i<numberOfOutputPoints;i++)
		lines->InsertCellPoint(i);

	profileData->SetPoints(points);
	profileData->SetLines(lines);

	// Add thickness to the resulting line.
	vtkTubeFilter *profileTubes = vtkTubeFilter::New();
	profileTubes->SetNumberOfSides(8);
	profileTubes->SetInput(profileData);
	profileTubes->SetRadius(.005);

	vtkPolyDataMapper *profileMapper = vtkPolyDataMapper::New();
	profileMapper->SetInputConnection(profileTubes->GetOutputPort());

	vtkActor *profile = vtkActor::New();
	profile->SetMapper(profileMapper);
	profile->GetProperty()->SetDiffuseColor(0,1,1);
	profile->GetProperty()->SetSpecular(.3);
	profile->GetProperty()->SetSpecularPower(30);


	// Now create the RenderWindow, Renderer and Interactor
	//
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);

	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors
	ren1->AddActor(glyph);
	ren1->AddActor(profile);

	renWin->SetSize(500, 500);

	// render the image
	//
	iren->Initialize();
	iren->Start();

	return 0 ;
}