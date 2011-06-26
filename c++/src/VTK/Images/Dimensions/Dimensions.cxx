#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkJPEGReader.h>

int main ( int argc, char *argv[] )
{
  //parse command line arguments
  if ( argc != 2 )
  {
    std::cout << "Required parameters: InputFilename" << std::endl;
    exit ( -1 );
  }

  vtkstd::string InputFilename = argv[1];

  //read JPG file
  vtkSmartPointer<vtkJPEGReader> Reader = vtkSmartPointer<vtkJPEGReader>::New();
  Reader->SetFileName ( InputFilename.c_str() );
  Reader->Update();
  
  vtkImageData* Image = Reader->GetOutput();
  
  int dims[3];
  Image->GetDimensions(dims);
  
  vtkstd::cout << "dims[0] = " << dims[0] << vtkstd::endl; //width
  vtkstd::cout << "dims[1] = " << dims[1] << vtkstd::endl; //height
  
  return 0;
}
