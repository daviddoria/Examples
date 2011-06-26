#include "vtkSmartPointer.h"

#include "vtkActor.h"
#include "vtkCommand.h"
#include "vtkConeSource.h"
#include "vtkGlyph3D.h"
#include "vtkInteractorEventRecorder.h"
#include "vtkPLOT3DReader.h"
#include "vtkPlaneWidget.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkProbeFilter.h"
#include "vtkProperty.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkStructuredGrid.h"
#include "vtkStructuredGridOutlineFilter.h"

#include "vtkTestUtilities.h"

// This does the actual work: updates the probe.
// Callback for the interaction
class vtkTPWCallback : public vtkCommand
{
public:
  static vtkTPWCallback *New() 
  { return new vtkTPWCallback; }
  virtual void Execute(vtkObject *caller, unsigned long, void*)
  {
    vtkPlaneWidget *planeWidget = reinterpret_cast<vtkPlaneWidget*>(caller);
    planeWidget->GetPolyData(this->PolyData);
    this->Actor->VisibilityOn();
  }
  vtkTPWCallback():PolyData(0),Actor(0) {}
  vtkPolyData *PolyData;
  vtkActor *Actor;
};

int TestPlaneWidget( int argc, char *argv[] )
{
  char* fname = 
    vtkTestUtilities::ExpandDataFileName(argc, argv, "Data/combxyz.bin");
  char* fname2 = 
    vtkTestUtilities::ExpandDataFileName(argc, argv, "Data/combq.bin");

  // Start by loading some data.
  //
  vtkSmartPointer<vtkPLOT3DReader> pl3d =
    vtkSmartPointer<vtkPLOT3DReader>::New();
  pl3d->SetXYZFileName(fname);
  pl3d->SetQFileName(fname2);
  pl3d->SetScalarFunctionNumber(100);
  pl3d->SetVectorFunctionNumber(202);
  pl3d->Update();

  delete [] fname;
  delete [] fname2;

  vtkSmartPointer<vtkPolyData> plane =
    vtkSmartPointer<vtkPolyData>::New();

  vtkSmartPointer<vtkProbeFilter> probe =
    vtkSmartPointer<vtkProbeFilter>::New();
  probe->SetInput(plane);
  probe->SetSource(pl3d->GetOutput());

  vtkSmartPointer<vtkPolyDataMapper> probeMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  probeMapper->SetInput(probe->GetPolyDataOutput());
  double tmp[2];
  pl3d->GetOutput()->GetScalarRange(tmp);
  probeMapper->SetScalarRange(tmp[0], tmp[1]);
  
  vtkSmartPointer<vtkActor> probeActor =
    vtkSmartPointer<vtkActor>::New();
  probeActor->SetMapper(probeMapper);
  probeActor->VisibilityOff();

  // An outline is shown for context.
  vtkSmartPointer<vtkStructuredGridOutlineFilter> outline =
    vtkSmartPointer<vtkStructuredGridOutlineFilter>::New();
  outline->SetInputConnection(pl3d->GetOutputPort());

  vtkSmartPointer<vtkPolyDataMapper> outlineMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  outlineMapper->SetInputConnection(outline->GetOutputPort());

  vtkSmartPointer<vtkActor> outlineActor =
    vtkSmartPointer<vtkActor>::New();
  outlineActor->SetMapper(outlineMapper);

  // Create the RenderWindow, Renderer and both Actors
  //
  vtkSmartPointer<vtkRenderer> ren1 =
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renWin =
    vtkSmartPointer<vtkRenderWindow>::New();
  renWin->AddRenderer(ren1);

  vtkSmartPointer<vtkRenderWindowInteractor> iren =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  iren->SetRenderWindow(renWin);

  // The SetInteractor method is how 3D widgets are associated with the render
  // window interactor. Internally, SetInteractor sets up a bunch of callbacks
  // using the Command/Observer mechanism (AddObserver()).
  vtkSmartPointer<vtkTPWCallback> myCallback =
    vtkSmartPointer<vtkTPWCallback>::New();
  myCallback->PolyData = plane;
  myCallback->Actor = probeActor;

  // The plane widget is used probe the dataset.
  //
  vtkSmartPointer<vtkPlaneWidget> planeWidget =
    vtkSmartPointer<vtkPlaneWidget>::New();
  planeWidget->SetInteractor(iren);
  planeWidget->SetInput(pl3d->GetOutput());
  planeWidget->NormalToXAxisOn();
  planeWidget->SetResolution(20);
  planeWidget->SetRepresentationToOutline();
  planeWidget->PlaceWidget();
  planeWidget->AddObserver(vtkCommand::InteractionEvent,myCallback);

  ren1->AddActor(probeActor);
  ren1->AddActor(outlineActor);

  // Add the actors to the renderer, set the background and size
  //
  ren1->SetBackground(0.1, 0.2, 0.4);
  renWin->SetSize(300, 300);

  // record events
  vtkSmartPointer<vtkInteractorEventRecorder> recorder =
    vtkSmartPointer<vtkInteractorEventRecorder>::New();
  recorder->SetInteractor(iren);
//  recorder->SetFileName("c:/record.log");
//  recorder->Record();
  recorder->ReadFromInputStringOn();
  recorder->SetInputString(TPWeventLog);

  // render the image
  //
  iren->Initialize();
  renWin->Render();
  recorder->Play();

  // Remove the observers so we can go interactive. Without this the "-I"
  // testing option fails.
  recorder->Off();

  iren->Start();

  return EXIT_SUCCESS;
}