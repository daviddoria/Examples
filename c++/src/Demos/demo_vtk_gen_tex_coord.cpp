/*
	Author: Chung Kai Lun Peter
	Email: hkpeterpeter@gmail.com
	File:
	    demo_vtk_gen_tex_coord.cpp
	Purpose:
		Ported from GenerateTextureCoords.tcl
*/
#include "project_config.h"

#include "vtkActor.h"
#include "vtkBMPReader.h"
#include "vtkDataSetMapper.h"
#include "vtkDelaunay3D.h"
#include "vtkPointSource.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkTexture.h"
#include "vtkTextureMapToCylinder.h"
#include "vtkTransformTextureCoords.h"

#include "vtkProperty.h"
#include "vtkCamera.h"


// Set VTK data root here..
// #define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"


int main(int argc, char** argv)
{
	// This example shows how to generate and manipulate texture coordinates.
	// A random cloud of points is generated and then triangulated with
	// vtkDelaunay3D. Since these points do not have texture coordinates,
	// we generate them with vtkTextureMapToCylinder.


	// Begin by generating 25 random points in the unit sphere.
	//
	vtkPointSource *sphere = vtkPointSource::New();
	sphere->SetNumberOfPoints(25);

	// Triangulate the points with vtkDelaunay3D. This generates a convex hull
	// of tetrahedron.
	//
	vtkDelaunay3D *del = vtkDelaunay3D::New();
	del->SetInputConnection(sphere->GetOutputPort());
	del->SetTolerance(0.01);

	// The triangulation has texture coordinates generated so we can map
	// a texture onto it.
	//
	vtkTextureMapToCylinder *tmapper = vtkTextureMapToCylinder::New();
	tmapper->SetInputConnection(del->GetOutputPort());
	tmapper->PreventSeamOn();

	// We scale the texture coordinate to get some repeat patterns.
	vtkTransformTextureCoords *xform = vtkTransformTextureCoords::New();
	xform->SetInputConnection(tmapper->GetOutputPort());
	xform->SetScale(4, 4, 1);

	// vtkDataSetMapper internally uses a vtkGeometryFilter to extract the
	// surface from the triangulation. The output (which is vtkPolyData) is
	// then passed to an internal vtkPolyDataMapper which does the
	// rendering.
	vtkDataSetMapper *mapper = vtkDataSetMapper::New();
	mapper->SetInputConnection(xform->GetOutputPort());

	// A texture is loaded using an image reader. Textures are simply images.
	// The texture is eventually associated with an actor.
	//
	vtkBMPReader *bmpReader = vtkBMPReader::New();
	bmpReader->SetFileName(VTK_DATA_ROOT "masonry.bmp");
	vtkTexture *atext = vtkTexture::New();
	atext->SetInputConnection(bmpReader->GetOutputPort());
	atext->InterpolateOn();
	vtkActor *triangulation = vtkActor::New();
	triangulation->SetMapper(mapper);
	triangulation->SetTexture(atext);

	// Create the standard rendering stuff.
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//
	ren1->AddActor(triangulation);
	ren1->SetBackground(1, 1, 1);
	renWin->SetSize(300, 300);
	renWin->Render();

	// render the image
	//
	renWin->Render();

	iren->Initialize();


	iren->Start();


	return 0 ;
}