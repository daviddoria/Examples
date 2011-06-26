#include <vtkPolyData.h>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>

int main ( int argc, char *argv[] )
{
  if ( argc != 2 )
    {
    std::cout << "Required parameters: Filename" << std::endl;
    return EXIT_FAILURE;
    }

  std::string inputFilename = argv[1];

  vtkSmartPointer<vtkPLYReader> plyReader = vtkSmartPointer<vtkPLYReader>::New();
  plyReader->SetFileName ( inputFilename.c_str() );
  plyReader->Update();
  
  vtkPolyData* data = plyReader->GetOutput();

  return EXIT_SUCCESS;
}
