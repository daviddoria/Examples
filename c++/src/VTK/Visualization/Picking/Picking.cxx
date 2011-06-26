#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkSphereSource.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkCellPicker.h>
#include <vtkImagePlaneWidget.h>
#include <vtkProperty.h>

int main ()
{

	 //Setup point coordinates
  double X[3] = {1.0, 0.0, 0.0};
  double Y[3] = {0.0, 0.0, 1.0};
  double Z[3] = {0.0, 0.0, 0.0};
 
  //Create points and add a vertex at each point. Really what you are doing is adding
  //cells to the polydata, and the cells only contain 1 element, so they are, by definition,
  //0-D topology (vertices).
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> Vertices = vtkSmartPointer<vtkCellArray>::New();
 
  for ( unsigned int i = 0; i < 3; ++i )
  {
    //Declare a variable to store the index of the point that gets added. This behaves just like an unsigned int.
    vtkIdType pid[1];
 
    //Add a point to the polydata and save its index, which we will use to create the vertex on that point.
    pid[0] = Points->InsertNextPoint ( X[i], Y[i], Z[i] );
 
    //create a vertex cell on the point that was just added.
    Vertices->InsertNextCell ( 1,pid );
  }
  //create a polydata object
  vtkSmartPointer<vtkPolyData> Polydata = vtkSmartPointer<vtkPolyData>::New();
 
  //set the points and vertices we created as the geometry and topology of the polydata
  Polydata->SetPoints ( Points );
  Polydata->SetVerts ( Vertices );

	//create a mapper
	vtkSmartPointer<vtkPolyDataMapper> Mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	Mapper->SetInput ( Polydata );

	// create an actor
	vtkSmartPointer<vtkActor> Actor = vtkSmartPointer<vtkActor>::New();
	Actor->SetMapper ( Mapper );

	// a renderer and render window
	vtkSmartPointer<vtkRenderer> Renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> RenderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	RenderWindow->AddRenderer ( Renderer );

	// an interactor
	vtkSmartPointer<vtkRenderWindowInteractor> RenderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	RenderWindowInteractor->SetRenderWindow ( RenderWindow );

	// add the actors to the scene
	Renderer->AddActor ( Actor );
	//Renderer->SetBackground ( 1,1,1 ); // Background color white
    Renderer->SetBackground ( 0,0,1 );
    
    
    vtkSmartPointer<vtkCellPicker> Picker = vtkSmartPointer<vtkCellPicker>::New();
    Picker->SetTolerance(0.005);

    vtkProperty* ipwProp = vtkProperty::New();

    vtkImagePlaneWidget* planeWidgetX = vtkImagePlaneWidget::New();
    planeWidgetX->SetInteractor(RenderWindowInteractor);
    planeWidgetX->SetKeyPressActivationValue('x');
    planeWidgetX->SetPicker(Picker);
    planeWidgetX->RestrictPlaneToVolumeOn();
    planeWidgetX->GetPlaneProperty()->SetColor(1,0,0);
    planeWidgetX->SetTexturePlaneProperty(ipwProp);
    planeWidgetX->TextureInterpolateOff();
    planeWidgetX->SetResliceInterpolateToNearestNeighbour();
    planeWidgetX->SetInput(Polydata);
    planeWidgetX->SetPlaneOrientationToXAxes();
    planeWidgetX->SetSliceIndex(32);
    planeWidgetX->DisplayTextOn();
    planeWidgetX->On();
    planeWidgetX->InteractionOff();
    planeWidgetX->InteractionOn();

	// render and interact
	RenderWindow->Render();
	RenderWindowInteractor->Start();

	return 0;
}
