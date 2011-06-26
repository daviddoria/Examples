#include <vtkImageData.h>
#include <vtkBMPReader.h>
#include <vtkSmartPointer.h>

int main ( int argc, char *argv[] )
{
  if ( argc != 2 )
    {
    cout << "Required parameters: Filename" << endl;
    return EXIT_FAILURE;
    }

  std::string inputFilename = argv[1];

  vtkSmartPointer<vtkBMPReader> bmpReader = vtkSmartPointer<vtkBMPReader>::New();
  bmpReader->SetFileName ( inputFilename.c_str() );

  vtkImageData* image = bmpReader->GetOutput();

  return EXIT_SUCCESS;
}
