#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkPLOT3DReader.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkStreamLine.h>
#include <vtkTestUtilities.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkStructuredGridOutlineFilter.h>
#include <vtkProperty.h>

#include <vtkTesting.h>

// The environment variable VTK_DATA_ROOT will be used.
// Make sure it is set accordingly!

int main(int argc, char *argv[])
{
  // Locate VTK_DATA_ROOT
  vtkSmartPointer<vtkTesting> testHelper =
    vtkSmartPointer<vtkTesting>::New();
  std::string dataRoot = testHelper->GetDataRoot();

  // Start by loading some data.
  std::string xyzFile = dataRoot + "/Data/" + "combxyz.bin";
  std::string qFile = dataRoot + "/Data/" + "combq.bin";
  vtkSmartPointer<vtkPLOT3DReader> pl3d =
    vtkSmartPointer<vtkPLOT3DReader>::New();
  pl3d->SetXYZFileName(xyzFile.c_str());
  pl3d->SetQFileName(qFile.c_str());
  pl3d->SetScalarFunctionNumber(100);
  pl3d->SetVectorFunctionNumber(202);
  pl3d->Update();

  // Streamline itself
  vtkSmartPointer<vtkStreamLine> streamLine =
    vtkSmartPointer<vtkStreamLine>::New();
  streamLine->SetInputConnection(pl3d->GetOutputPort());
  streamLine->SetStartPosition(2,-2,30);
  streamLine->SetMaximumPropagationTime(200);
  streamLine->SetIntegrationStepLength(.2);
  streamLine->SetStepLength(.001);
  streamLine->SetNumberOfThreads(1);
  streamLine->SetIntegrationDirectionToForward();
  streamLine->VorticityOn();

  vtkSmartPointer<vtkPolyDataMapper> streamLineMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  streamLineMapper->SetInputConnection(streamLine->GetOutputPort());

  vtkSmartPointer<vtkActor> streamLineActor =
    vtkSmartPointer<vtkActor>::New();
  streamLineActor->SetMapper(streamLineMapper);
  streamLineActor->VisibilityOn();

  // Outline-Filter for the grid
  vtkSmartPointer<vtkStructuredGridOutlineFilter> outline =
    vtkSmartPointer<vtkStructuredGridOutlineFilter>::New();
  outline->SetInputConnection(pl3d->GetOutputPort());

  vtkSmartPointer<vtkPolyDataMapper> outlineMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  outlineMapper->SetInputConnection(outline->GetOutputPort());

  vtkSmartPointer<vtkActor> outlineActor =
    vtkSmartPointer<vtkActor>::New();
  outlineActor->SetMapper(outlineMapper);
  outlineActor->GetProperty()->SetColor(1, 1, 1);

  // Create the RenderWindow, Renderer and Actors
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
    vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
  interactor->SetInteractorStyle(style);

  renderer->AddActor(streamLineActor);
  renderer->AddActor(outlineActor);

  // Add the actors to the renderer, set the background and size
  renderer->SetBackground(0.1, 0.2, 0.4);
  renderWindow->SetSize(300, 300);
  interactor->Initialize();
  renderWindow->Render();

  interactor->Start();

  return EXIT_SUCCESS;
}