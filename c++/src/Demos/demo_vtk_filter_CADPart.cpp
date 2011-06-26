/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_filter_CADPart.cpp
	Purpose:
		FilterCADPart.tcl
*/
#include "project_config.h"

#include "vtkLODActor.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSTLReader.h"
#include "vtkShrinkPolyData.h"

#include "vtkProperty.h"
#include "vtkCamera.h"


// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	//
	// This simple example shows how to do simple filtering in a pipeline.
	// See CADPart.tcl and Cylinder.tcl for related information.
	//
	// We start off by loading some Tcl modules. One is the basic VTK library;
	// the second is a package for rendering, and the last includes a set
	// of color definitions.
	//

	// This creates a polygonal cylinder model with eight circumferential facets.
	//
	vtkSTLReader *part = vtkSTLReader::New();
	part->SetFileName(VTK_DATA_ROOT  "42400-IDGH.stl");

	// A filter is a module that takes at least one input and produces at
	// least one output. The SetInput and GetOutput methods are used to do
	// the connection. What is returned by GetOutput is a particulat dataset
	// type. If the type is compatible with the SetInput method, then the
	// filters can be connected together.
	//
	// Here we add a filter that computes surface normals from the geometry.
	//
	vtkShrinkPolyData *shrink = vtkShrinkPolyData::New();
	shrink->SetInputConnection(part->GetOutputPort());
	shrink->SetShrinkFactor(0.85);

	// The mapper is responsible for pushing the geometry into the graphics
	// library. It may also do color mapping, if scalars or other attributes
	// are defined.
	//
	vtkPolyDataMapper *partMapper = vtkPolyDataMapper::New();
	partMapper->SetInputConnection(shrink->GetOutputPort());

	// The LOD actor is a special type of actor. It will change appearance in
	// order to render faster. At the highest resolution, it renders ewverything
	// just like an actor. The middle level is a point cloud, and the lowest
	// level is a simple bounding box.
	vtkLODActor *partActor = vtkLODActor::New();
	partActor->SetMapper(partMapper);
	partActor->GetProperty()->SetColor(0.9,0.9,0.9);
	partActor->RotateX( 30.0);
	partActor->RotateY(-45.0);

	// Create the graphics structure. The renderer renders into the
	// render window. The render window interactor captures mouse events
	// and will perform appropriate camera or actor manipulation
	// depending on the nature of the events.
	//
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//
	ren1->AddActor(partActor);
	ren1->SetBackground(0.1, 0.2, 0.4);
	renWin->SetSize(200, 200);



	// This starts the event loop and as a side effect causes an initial render.
	iren->Initialize();

	// We'll zoom in a little by accessing the camera and invoking a "Zoom"
	// method on it.
	ren1->ResetCamera();
	ren1->GetActiveCamera()->Zoom(1.5);
	renWin->Render();




	iren->Start();

	return 0 ;
}