/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_strips.cpp
	Purpose:
	    Ported from CreateStrip.tcl

*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkCellArray.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"

#include "vtkProperty.h"
#include "vtkCamera.h"

// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	// First we'll create some points.
	//
	vtkPoints *points = vtkPoints::New();
	points->InsertPoint(0, 0.0, 0.0, 0.0);
	points->InsertPoint(1, 0.0, 1.0, 0.0);
	points->InsertPoint(2, 1.0, 0.0, 0.0);
	points->InsertPoint(3, 1.0, 1.0, 0.0);
	points->InsertPoint(4, 2.0, 0.0, 0.0);
	points->InsertPoint(5, 2.0, 1.0, 0.0);
	points->InsertPoint(6, 3.0, 0.0, 0.0);
	points->InsertPoint(7, 3.0, 1.0, 0.0);

	// The cell array can be thought of as a connectivity list.
	// Here we specify the number of points followed by that number of
	// point ids. This can be repeated as many times as there are
	// primitives in the list.
	//
	vtkCellArray *strips = vtkCellArray::New();
	strips->InsertNextCell(8);
	strips->InsertCellPoint(0);
	strips->InsertCellPoint(1);
	strips->InsertCellPoint(2);
	strips->InsertCellPoint(3);
	strips->InsertCellPoint(4);
	strips->InsertCellPoint(5);
	strips->InsertCellPoint(6);
	strips->InsertCellPoint(7);
	vtkPolyData *profile = vtkPolyData::New();
	profile->SetPoints(points);
	profile->SetStrips(strips);

	vtkPolyDataMapper *map = vtkPolyDataMapper::New();
	map->SetInput(profile);

	vtkActor *strip = vtkActor::New();
	strip->SetMapper(map);
	strip->GetProperty()->SetColor(0.3800, 0.7000, 0.1600);

	// Create the usual rendering stuff.
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//
	ren1->AddActor(strip);

	ren1->SetBackground(1, 1, 1);
	renWin->SetSize(250, 250);
	renWin->Render();

	// render the image
	//

	iren->Initialize();
	iren->Start();




	return 0 ;
}