#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkPolyData.h"
#include "vtkXMLPolyDataReader.h"
#include <vtkSmartPointer.h>

int main (int argc, char **argv)
{
  if(argc != 2)
  {
    vtkstd::cout << "Required arguments: Filename" << vtkstd::endl;
    exit(-1);
  }
 
  vtkstd::string Filename = argv[1];
 
  vtkstd::cout << "Reading " << Filename << vtkstd::endl;
  
  vtkSmartPointer<vtkXMLPolyDataReader> Reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  Reader->SetFileName(Filename.c_str());
  Reader->Update();
	
  //vtkPolyData* Polydata = Reader->GetOutput();
  vtkSmartPointer<vtkPolyData> Polydata = Reader->GetOutput();
  
  //create a mapper
  vtkSmartPointer<vtkPolyDataMapper> Mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  Mapper->SetInput(Polydata);

  // create an actor
  vtkSmartPointer<vtkActor> Actor = vtkSmartPointer<vtkActor>::New();
  Actor->SetMapper(Mapper);

  // a renderer and render window
  vtkSmartPointer<vtkRenderer> Renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> RenderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  RenderWindow->AddRenderer(Renderer);

  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> RenderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  RenderWindowInteractor->SetRenderWindow(RenderWindow);

  // add the actors to the scene
  Renderer->AddActor(Actor);
  Renderer->SetBackground(1,1,1); // Background color white

  // render an image (lights and cameras are created automatically)
  RenderWindow->Render();

  // begin mouse interaction
  RenderWindowInteractor->Start();

  return 0;
}
