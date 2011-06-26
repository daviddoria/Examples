#include <vtkImageData.h>
#include <vtkTIFFReader.h>
#include <vtkSmartPointer.h>

int main ( int argc, char *argv[] )
{
  if ( argc != 2 )
    {
    cout << "Required parameters: Filename" << endl;
    return EXIT_FAILURE;
    }

  std::string inputFilename = argv[1];

  vtkSmartPointer<vtkTIFFReader> tiffReader = vtkSmartPointer<vtkTIFFReader>::New();
  tiffReader->SetFileName ( inputFilename.c_str() );

  vtkImageData* image  = tiffReader->GetOutput();

  return EXIT_SUCCESS;
}
