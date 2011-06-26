#include <vtkSmartPointer.h>

#include <vtkKWApplication.h>
#include <vtkKWWindowBase.h>
#include <vtkKWLabel.h>
#include <vtkKWFrame.h>

int main(int argc, char *argv[])
{
  Tcl_Interp *interp = vtkKWApplication::InitializeTcl(argc, argv, &cerr);
  
  vtkSmartPointer<vtkKWApplication> app = 
      vtkSmartPointer<vtkKWApplication>::New();
  app->SetName("KWHelloWorldExample");

  app->SetHelpDialogStartingPage("http://www.kwwidgets.org");

  // Add a window
  vtkSmartPointer<vtkKWWindowBase> win = 
      vtkSmartPointer<vtkKWWindowBase>::New();
  win->SupportHelpOn();
  app->AddWindow(win);
  win->Create();

  // Add a label, attach it to the view frame, and pack
  
  vtkSmartPointer<vtkKWLabel> hello_label = 
      vtkSmartPointer<vtkKWLabel>::New();
  hello_label->SetParent(win->GetViewFrame());
  hello_label->Create();
  hello_label->SetText("Hello, World!");
  app->Script("pack %s -side left -anchor c -expand y", 
              hello_label->GetWidgetName());
  hello_label->Delete();

  win->Display();
    
  app->Start(argc, argv);
  int ret = app->GetExitStatus();
  
  win->Close();

  return ret;
}
