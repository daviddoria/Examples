#include "vtkKWApplication.h"
#include "vtkKWWindowBase.h"
#include "vtkKWLabel.h"
#include "vtkKWFrame.h"

#include <vtksys/SystemTools.hxx>
#include <vtksys/CommandLineArguments.hxx>

int main(int argc, char *argv[])
{
  Tcl_Interp *interp = vtkKWApplication::InitializeTcl(argc, argv, &cerr);
  
  vtkKWApplication *app = vtkKWApplication::New();
  app->SetName("KWHelloWorldExample");

  
  // Set a help link. Can be a remote link (URL), or a local file

  // vtksys::SystemTools::GetFilenamePath(__FILE__) + "/help.html";
  app->SetHelpDialogStartingPage("http://www.kwwidgets.org");

  // Add a window
  // Set 'SupportHelp' to automatically add a menu entry for the help link

  vtkKWWindowBase *win = vtkKWWindowBase::New();
  win->SupportHelpOn();
  app->AddWindow(win);
  win->Create();

  // Add a label, attach it to the view frame, and pack
  
  vtkKWLabel *hello_label = vtkKWLabel::New();
  hello_label->SetParent(win->GetViewFrame());
  hello_label->Create();
  hello_label->SetText("Hello, World!");
  app->Script("pack %s -side left -anchor c -expand y", 
              hello_label->GetWidgetName());
  hello_label->Delete();

  // Start the application
  // If --test was provided, do not enter the event loop and run this example
  // as a non-interactive test for software quality purposes.

  int ret = 0;
  win->Display();
    
  app->Start(argc, argv);
  ret = app->GetExitStatus();
  
  win->Close();

  // Deallocate and exit

  win->Delete();
  app->Delete();
  
  return ret;
}
