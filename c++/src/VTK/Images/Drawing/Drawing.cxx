#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkJPEGReader.h>
#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include <vtkImageViewer2.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageBlend.h>

int main ( int argc, char* argv[] )
{
  // Verify input arguments
  if ( argc != 2 )
    {
    std::cout << "Required parameters: InputFilename" << std::endl << "Exiting..." << std::endl;
    return EXIT_FAILURE;
    }

  // Parse input arguments
  std::string inputFilename = argv[1];
  
  // Read the image
  vtkSmartPointer<vtkJPEGReader> jPEGReader = 
    vtkSmartPointer<vtkJPEGReader>::New();
  jPEGReader->SetFileName ( inputFilename.c_str() );
  jPEGReader->Update();
  vtkImageData* image = jPEGReader->GetOutput();

  // Draw a circle
  vtkSmartPointer<vtkImageCanvasSource2D> drawing = 
    vtkSmartPointer<vtkImageCanvasSource2D>::New();
  drawing->SetScalarTypeToUnsignedChar();
  drawing->SetExtent(image->GetExtent());
  drawing->SetDrawColor(255.0, 0.0, 0.0, 0.5);
  drawing->DrawCircle(10, 10, 5);
  drawing->Update();
  
  // Combine the images (blend takes multiple connections on the 0th input port)
  vtkSmartPointer<vtkImageBlend> blend = 
    vtkSmartPointer<vtkImageBlend>::New();
  blend->AddInputConnection(jPEGReader->GetOutputPort()); //can't do this because 'input' is not an algorithm output
  blend->AddInputConnection(drawing->GetOutputPort());
  blend->SetOpacity(0,.5);
  blend->SetOpacity(1,.5);
  blend->Update();
  
  // Display the result
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  vtkSmartPointer<vtkImageViewer2> imageViewer = 
    vtkSmartPointer<vtkImageViewer2>::New();
  imageViewer->SetInput(blend->GetOutput());
  imageViewer->SetupInteractor(renderWindowInteractor);
  imageViewer->GetRenderer()->ResetCamera();
  imageViewer->GetRenderer()->SetBackground(1,0,0); //red
  renderWindowInteractor->Initialize();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
