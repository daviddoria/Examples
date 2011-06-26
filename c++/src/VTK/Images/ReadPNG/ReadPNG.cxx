#include <vtkImageData.h>
#include <vtkPNGReader.h>
#include <vtkPNGWriter.h>
#include <vtkSmartPointer.h>

int main ( int argc, char *argv[] )
{
  //parse command line arguments
	if ( argc != 3 )
	{
		std::cout << "Required parameters: Filename" << std::endl;
		exit ( -1 );
	}

	std::string InputFilename = argv[1];
  std::string OutputFilename = argv[2];

  //read PNG file
	vtkSmartPointer<vtkPNGReader> Reader = vtkSmartPointer<vtkPNGReader>::New();
	Reader->SetFileName ( InputFilename.c_str() );

	vtkImageData* Image  = Reader->GetOutput();

  //write PNG file
  vtkSmartPointer<vtkPNGWriter> Writer = vtkSmartPointer<vtkPNGWriter>::New();
  Writer->SetFileName ( OutputFilename.c_str() );
  Writer->SetInput(Image);
  Writer->Write();

	return 0;
}
