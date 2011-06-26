#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkJPEGReader.h>
#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include <vtkImageViewer2.h>


int main ( int argc, char* argv[] )
{
  //parse input arguments
  if ( argc != 3 )
    {
      std::cout << "Required parameters: InputFilename OutputFilename" << std::endl << "Exiting..." << std::endl;
    exit ( -1 );
    }

  vtkstd::string InputFilename = argv[1];
  vtkstd::string OutputFilename = argv[2];

  //read the image
  vtkSmartPointer<vtkJPEGReader> JPEGReader = vtkSmartPointer<vtkJPEGReader>::New();
  JPEGReader->SetFileName ( InputFilename.c_str() );
  JPEGReader->Update();
  
  vtkImageData* image = JPEGReader->GetOutput();
  //set the pixel (10,10) to red
  //measured from lower left corner
  image->SetScalarComponentFromDouble(10, 10, 0, 0, 255.0);
  image->SetScalarComponentFromDouble(10, 10, 0, 1, 0.0);
  image->SetScalarComponentFromDouble(10, 10, 0, 2, 0.0);
  
  vtkSmartPointer<vtkRenderWindowInteractor> RenderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();

  vtkSmartPointer<vtkImageViewer2> ImageViewer = vtkSmartPointer<vtkImageViewer2>::New();
  ImageViewer->SetInput(image);
  ImageViewer->SetupInteractor(RenderWindowInteractor);
  ImageViewer->GetRenderer()->ResetCamera();
  ImageViewer->GetRenderer()->SetBackground(1,0,0); //red
  
  RenderWindowInteractor->Initialize();
  RenderWindowInteractor->Start();
  
  return 0 ;
}
