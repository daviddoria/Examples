#include <vtkImageData.h>
#include <vtkPNMReader.h>
#include <vtkSmartPointer.h>

int main ( int argc, char *argv[] )
{
  if ( argc != 2 )
    {
    cout << "Required parameters: Filename" << endl;
    return EXIT_FAILURE;
    }

  std::string inputFilename = argv[1];

  vtkSmartPointer<vtkPNMReader> pnmReader = vtkSmartPointer<vtkPNMReader>::New();
  pnmReader->SetFileName ( inputFilename.c_str() );
  pnmReader->Update();
  
  vtkImageData* image = pnmReader->GetOutput();

  return EXIT_SUCCESS;
}
