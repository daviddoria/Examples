#include <vtkFileOutputWindow.h>
#include <vtkOutputWindow.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkSmartPointer.h>
 
int main( int argc, char *argv[] )
{
  vtkSmartPointer<vtkFileOutputWindow> fileOutputWindow = 
      vtkSmartPointer<vtkFileOutputWindow>::New();
  fileOutputWindow->SetFileName( "output.txt" );
 
  vtkOutputWindow* outputWindow = vtkOutputWindow::GetInstance();
  if ( outputWindow )
    {
    outputWindow->SetInstance( fileOutputWindow );
    }
 
  //this causes an error - file name not specified
  vtkSmartPointer<vtkXMLPolyDataReader> reader = 
      vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->Update();
 
  return EXIT_SUCCESS;
}