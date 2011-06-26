#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkJPEGReader.h>
#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include <vtkImageViewer2.h>
#include <vtkImageBlend.h>

int main ( int argc, char* argv[] )
{
  //parse input arguments
  if ( argc != 3 )
    {
    std::cout << "Required parameters: Input1Filename Input2Filename" << std::endl;
    exit ( -1 );
    }

  vtkstd::string Input1Filename = argv[1];
  vtkstd::string Input2Filename = argv[2];

  //read the images
  vtkSmartPointer<vtkJPEGReader> JPEGReader1 = vtkSmartPointer<vtkJPEGReader>::New();
  JPEGReader1->SetFileName ( Input1Filename.c_str() );
  JPEGReader1->Update();
  
  vtkSmartPointer<vtkJPEGReader> JPEGReader2 = vtkSmartPointer<vtkJPEGReader>::New();
  JPEGReader2->SetFileName ( Input2Filename.c_str() );
  JPEGReader2->Update();

  //combine the images (blend takes multiple connections on the 0th input port)
  vtkSmartPointer<vtkImageBlend> blend = vtkSmartPointer<vtkImageBlend>::New();
  blend->AddInputConnection(JPEGReader1->GetOutputPort());
  blend->AddInputConnection(JPEGReader2->GetOutputPort());
  blend->SetOpacity(0,.5);
  blend->SetOpacity(1,.5);
  blend->Update();
  
  //display the result
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();

  vtkSmartPointer<vtkImageViewer2> imageViewer = vtkSmartPointer<vtkImageViewer2>::New();
  imageViewer->SetInputConnection(blend->GetOutputPort());
  imageViewer->SetupInteractor(renderWindowInteractor);
  imageViewer->GetRenderer()->ResetCamera();
  imageViewer->GetRenderer()->SetBackground(1,0,0); //red
  
  renderWindowInteractor->Initialize();
  renderWindowInteractor->Start();
  
  return 0 ;
}
