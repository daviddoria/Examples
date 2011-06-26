#include <vtkSmartPointer.h>
#include <vtkTimerLog.h>

int main(int argc, char* argv[])
{
  vtkSmartPointer<vtkTimerLog> timerLog = 
      vtkSmartPointer<vtkTimerLog>::New();
  
  cout << "Current time: " << timerLog->GetUniversalTime() << endl;
  
  timerLog->MarkEvent("opened file");
  
  timerLog->MarkEvent("did other stuff");
  
  cout << "Timer log:" << *timerLog << endl;
  
  return EXIT_SUCCESS;
}