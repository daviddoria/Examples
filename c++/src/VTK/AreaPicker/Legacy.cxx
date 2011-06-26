#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkInteractorStyleTrackball.h>
#include <vtkAreaPicker.h>
#include <vtkCallbackCommand.h>

void PickCallbackFunction ( vtkObject* caller, long unsigned int eventId, void* clientData, void* callData );

int main()
{
  //create a set of points
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
  vtkIdType pid[1];
  pid[0] = points->InsertNextPoint ( 1.0, 0.0, 0.0 );
  vertices->InsertNextCell ( 1,pid );
  pid[0] = points->InsertNextPoint ( 0.0, 0.0, 0.0 );
  vertices->InsertNextCell ( 1,pid );
  pid[0] = points->InsertNextPoint ( 0.0, 1.0, 0.0 );
  vertices->InsertNextCell ( 1,pid );

  //create a polydata
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints ( points );
  polydata->SetVerts ( vertices );
  
  //create a mapper
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInput(polydata);

  // create an actor
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkAreaPicker> areaPicker = vtkSmartPointer<vtkAreaPicker>::New();
  
  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
  renderWindowInteractor->SetPicker(areaPicker);
  // add the actors to the scene
  renderer->AddActor(actor);
  renderer->SetBackground(1,1,1); // Background color white

  // render an image (lights and cameras are created automatically)
  renderWindow->Render();
  
  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New(); //like paraview
  
  renderWindowInteractor->SetInteractorStyle( style );
    
  vtkSmartPointer<vtkCallbackCommand> pickCallback = vtkSmartPointer<vtkCallbackCommand>::New();
  pickCallback->SetCallback ( PickCallbackFunction );

  renderWindowInteractor->AddObserver ( vtkCommand::PickEvent, pickCallback );
    
  // begin mouse interaction
  renderWindowInteractor->Start();
  
  return 0;
}


void PickCallbackFunction ( vtkObject* caller, long unsigned int eventId, void* clientData, void* callData )
{
  vtkstd::cout << "Pick." << vtkstd::endl;
}
