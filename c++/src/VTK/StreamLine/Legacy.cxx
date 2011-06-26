#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkPLOT3DReader.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkStreamLine.h>
#include <vtkStructuredGrid.h>
#include <vtkTestUtilities.h>

int main( int argc, char *argv[] )
{
  vtkstd::string xyzFileName = "/home/doriad/src/VTKData/Data/combxyz.bin";
  vtkstd::string qFileName = "/home/doriad/src/VTKData/Data/combq.bin";

  // Start by loading some data.
  //
  vtkSmartPointer<vtkPLOT3DReader> pl3d = 
      vtkSmartPointer<vtkPLOT3DReader>::New();
  pl3d->SetXYZFileName(xyzFileName.c_str());
  pl3d->SetQFileName(qFileName.c_str());
  pl3d->SetScalarFunctionNumber(100);
  pl3d->SetVectorFunctionNumber(202);
  pl3d->Update();

  vtkSmartPointer<vtkPolyData> seeds = 
      vtkSmartPointer<vtkPolyData>::New();

  vtkSmartPointer<vtkStreamLine> streamer = vtkSmartPointer<vtkStreamLine>::New();
  streamer->SetInputConnection(pl3d->GetOutputPort());
  streamer->SetSource(seeds);
  streamer->SetMaximumPropagationTime(100);
  streamer->SetIntegrationStepLength(.2);
  streamer->SetStepLength(.001);
  streamer->SetNumberOfThreads(1);
  streamer->SetIntegrationDirectionToForward();
  streamer->VorticityOn();
  
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(streamer->GetOutputPort());
  
  vtkSmartPointer<vtkActor> streamline = vtkSmartPointer<vtkActor>::New();
  streamline->SetMapper(mapper);
  streamline->VisibilityOff();

  // Create the RenderWindow, Renderer and both Actors
  //
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> interactor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  renderer->AddActor(streamline);

  // Add the actors to the renderer, set the background and size
  //
  renderer->SetBackground(0.1, 0.2, 0.4);
  renderWindow->SetSize(300, 300);

  interactor->Initialize();
  renderWindow->Render();

  interactor->Start();

  return EXIT_SUCCESS;
}