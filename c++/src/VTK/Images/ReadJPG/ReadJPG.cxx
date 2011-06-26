#include <vtkImageData.h>
#include <vtkJPEGReader.h>
#include <vtkJPEGWriter.h>
#include <vtkSmartPointer.h>

int main ( int argc, char *argv[] )
{
  //parse command line arguments
	if ( argc != 3 )
	{
      std::cout << "Required parameters: InputFilename OutputFilename" << std::endl;
      exit ( -1 );
	}

  vtkstd::string InputFilename = argv[1];
  vtkstd::string OutputFilename = argv[2];

  //read JPG file
  vtkSmartPointer<vtkJPEGReader> Reader = vtkSmartPointer<vtkJPEGReader>::New();
  Reader->SetFileName ( InputFilename.c_str() );

  vtkImageData* Image = Reader->GetOutput();

  //write JPG file
  vtkSmartPointer<vtkJPEGWriter> Writer = vtkSmartPointer<vtkJPEGWriter>::New();
  Writer->SetFileName ( InputFilename.c_str() );
  Writer->SetInput(Image);
  Writer->Write();

  return 0;
}
