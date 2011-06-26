#include <vtkJPEGReader.h>
#include <vtkJPEGWriter.h>
#include <vtkSmartPointer.h>
 
int main ( int argc, char *argv[] )
{
  //parse command line arguments
  if ( argc < 3 )
    {
    std::cout << "Usage: " << argv[0]
              << " InputFilename(.jpg) OutputFilename(.jpg)" << std::endl;
    return EXIT_FAILURE;
    }
 
  vtkstd::string inputFilename = argv[1];
  vtkstd::string outputFilename = argv[2];
 
  // Read JPG file
  vtkSmartPointer<vtkJPEGReader> reader =
    vtkSmartPointer<vtkJPEGReader>::New();
  reader->SetFileName ( inputFilename.c_str() );
 
  // Write JPG file
  vtkSmartPointer<vtkJPEGWriter> writer =
    vtkSmartPointer<vtkJPEGWriter>::New();
  writer->SetFileName ( outputFilename.c_str() );
  writer->SetInputConnection(reader->GetOutputPort());
  writer->Write();
 
  return EXIT_SUCCESS;
}