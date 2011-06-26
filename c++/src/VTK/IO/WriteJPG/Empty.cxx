#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkJPEGWriter.h>

int main ( int argc, char *argv[] )
{
  if ( argc != 2 )
    {
    cout << "Required parameters: OutputFilename.jpg" << endl;
    return EXIT_FAILURE;
    }

  std::string outputFilename = argv[1];
  
  vtkSmartPointer<vtkImageData> image =
      vtkSmartPointer<vtkImageData>::New();
  int extent[6] = {0, 10, 0, 5, 0, 0};
  image->SetExtent(extent);
  image->SetScalarTypeToUnsignedChar();
  
  cout << "number of points: " << image->GetNumberOfPoints() << endl;
  
  vtkSmartPointer<vtkJPEGWriter> writer = 
      vtkSmartPointer<vtkJPEGWriter>::New();
  writer->SetFileName(outputFilename.c_str());
  //writer->SetInputConnection(image->GetProducerPort());
  writer->SetInput(image);
  writer->Write();

  return EXIT_SUCCESS;
}