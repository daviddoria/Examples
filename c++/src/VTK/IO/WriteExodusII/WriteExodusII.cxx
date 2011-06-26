#include <vtkSmartPointer.h>
#include <vtkExodusIIWriter.h>
#include <vtkTimeSourceExample.h>
 
int main ( int argc, char *argv[] )
{
  if ( argc != 2 )
    {
    cout << "Required parameters: OutputFilename.exii" << endl;
    return EXIT_FAILURE;
    }
 
  std::string outputFilename = argv[1];
 
  vtkSmartPointer<vtkTimeSourceExample> timeSource = 
    vtkSmartPointer<vtkTimeSourceExample>::New();
 
  vtkSmartPointer<vtkExodusIIWriter> exodusWriter = 
    vtkSmartPointer<vtkExodusIIWriter>::New();
  exodusWriter->SetFileName(outputFilename.c_str());
  exodusWriter->SetInputConnection (timeSource->GetOutputPort());
  exodusWriter->WriteAllTimeStepsOn ();
  exodusWriter->Write();
 
  return EXIT_SUCCESS;
}