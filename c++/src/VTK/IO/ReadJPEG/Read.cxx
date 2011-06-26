#include <vtkSmartPointer.h>
#include <vtkJPEGReader.h>
#include <vtkImageData.h>
 
int main ( int argc, char *argv[] )
{
  //parse command line arguments
  if ( argc < 2 )
    {
    std::cout << "Usage: " << argv[0]
              << " InputFilename(.jpg)" << std::endl;
    return EXIT_FAILURE;
    }
 
  vtkstd::string inputFilename = argv[1];
   
  // Read JPG file
  vtkSmartPointer<vtkJPEGReader> reader =
    vtkSmartPointer<vtkJPEGReader>::New();
  reader->SetFileName ( inputFilename.c_str() );
  reader->Update();
  vtkSmartPointer<vtkImageData> image = reader->GetOutput();
  
  // Get dimensions of the image
  int* dims = image->GetDimensions();
  
  // Access elements of the image
  for (int row = 0; row < dims[1]; row++)
    {
    for (int col = 0; col < dims[0]; col++)
      {
      //double* pixel = static_cast<double*>(image->GetScalarPointer(row,col,0));
      //std::cout << "(" << row << " , " << col << ") = (" << pixel[0] << " , " << pixel[1] << ")";
    
      unsigned char* pixel = static_cast<unsigned char*>(image->GetScalarPointer(col,row,0));
      std::cout << "(" << row << " , " << col << ") = (" << int(pixel[0]) << " , " << int(pixel[1]) << ")" << std::endl;
      }
    }
    
  return EXIT_SUCCESS;
}
