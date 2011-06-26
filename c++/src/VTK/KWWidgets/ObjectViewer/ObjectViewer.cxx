#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkKWApplication.h>
#include <vtkKWFrameWithLabel.h>
#include <vtkKWRenderWidget.h>
#include <vtkKWSurfaceMaterialPropertyWidget.h>
#include <vtkKWWindow.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkKWSimpleAnimationWidget.h>
#include <vtkSphereSource.h>

#include <vtksys/SystemTools.hxx>
#include <vtksys/CommandLineArguments.hxx>

int main(int argc, char *argv[])
{
  // Initialize Tcl
  Tcl_Interp *interp = vtkKWApplication::InitializeTcl(argc, argv, &cerr);

  vtkSmartPointer<vtkKWApplication> app = 
      vtkSmartPointer<vtkKWApplication>::New();
  app->SetName("KWPolygonalObjectViewerExample");
  
  app->SetHelpDialogStartingPage("http://www.kwwidgets.org");

  // Add a window
  vtkSmartPointer<vtkKWWindow> win = 
      vtkSmartPointer<vtkKWWindow>::New();
  win->SupportHelpOn();
  app->AddWindow(win);
  win->Create();
  win->SecondaryPanelVisibilityOff();

  // Add a render widget, attach it to the view frame, and pack
  
  vtkSmartPointer<vtkKWRenderWidget> rw = 
      vtkSmartPointer<vtkKWRenderWidget>::New();
  rw->SetParent(win->GetViewFrame());
  rw->Create();

  app->Script("pack %s -expand y -fill both -anchor c -expand y", 
              rw->GetWidgetName());

  // Create a 3D object reader

  vtkSmartPointer<vtkSphereSource> sphereSource =
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  // Create the mapper and actor

  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  // Add the actor to the scene

  rw->AddViewProp(actor);
  rw->ResetCamera();

  // Create a material property editor

  vtkSmartPointer<vtkKWSurfaceMaterialPropertyWidget> mat_prop_widget = 
    vtkSmartPointer<vtkKWSurfaceMaterialPropertyWidget>::New();
  mat_prop_widget->SetParent(win->GetMainPanelFrame());
  mat_prop_widget->Create();
  mat_prop_widget->SetPropertyChangedCommand(rw, "Render");
  mat_prop_widget->SetPropertyChangingCommand(rw, "Render");

  mat_prop_widget->SetProperty(actor->GetProperty());

  app->Script("pack %s -side top -anchor nw -expand n -fill x",
              mat_prop_widget->GetWidgetName());

  // Create a simple animation widget

  vtkSmartPointer<vtkKWFrameWithLabel> animation_frame = 
      vtkSmartPointer<vtkKWFrameWithLabel>::New();
  animation_frame->SetParent(win->GetMainPanelFrame());
  animation_frame->Create();
  animation_frame->SetLabelText("Movie Creator");

  app->Script("pack %s -side top -anchor nw -expand n -fill x -pady 2",
              animation_frame->GetWidgetName());

  vtkSmartPointer<vtkKWSimpleAnimationWidget> animation_widget = 
    vtkSmartPointer<vtkKWSimpleAnimationWidget>::New();
  animation_widget->SetParent(animation_frame->GetFrame());
  animation_widget->Create();
  animation_widget->SetRenderWidget(rw);
  animation_widget->SetAnimationTypeToCamera();

  app->Script("pack %s -side top -anchor nw -expand n -fill x",
              animation_widget->GetWidgetName());

  
  win->Display();
  app->Start(argc, argv);
  int ret = app->GetExitStatus();
  win->Close();

  
  return ret;
}
