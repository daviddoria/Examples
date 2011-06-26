#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkJPEGReader.h>
#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkImageViewer2.h>
#include <vtkImageCheckerboard.h>

int main ( int argc, char* argv[] )
{
  //Verify input arguments
  if ( argc != 3 )
    {
    cout << "Required parameters: Filename1 Filename2" << endl << "Exiting..." << endl;
    exit ( -1 );
    }

  //Parse input arguments
  vtkstd::string inputFilename1 = argv[1];
  vtkstd::string inputFilename2 = argv[2];

  //Read the images
  vtkSmartPointer<vtkJPEGReader> jPEGReader1 = vtkSmartPointer<vtkJPEGReader>::New();
  jPEGReader1->SetFileName ( inputFilename1.c_str() );
  jPEGReader1->Update();
  
  vtkSmartPointer<vtkJPEGReader> jPEGReader2 = vtkSmartPointer<vtkJPEGReader>::New();
  jPEGReader2->SetFileName ( inputFilename2.c_str() );
  jPEGReader2->Update();
  
  vtkSmartPointer<vtkImageCheckerboard> checkerboardFilter = vtkSmartPointer<vtkImageCheckerboard>::New();
  //checkerboardFilter->AddInputConnection(jPEGReader1->GetOutputPort());
  //checkerboardFilter->AddInputConnection(jPEGReader2->GetOutputPort());
  checkerboardFilter->SetInput(0, jPEGReader1->GetOutput());
  checkerboardFilter->SetInput(1, jPEGReader2->GetOutput());
  checkerboardFilter->SetNumberOfDivisions(2, 2, 2);
  checkerboardFilter->Update();
  
      
  // Visualize
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();

  vtkSmartPointer<vtkImageViewer2> imageViewer = vtkSmartPointer<vtkImageViewer2>::New();
  imageViewer->SetInputConnection(checkerboardFilter->GetOutputPort());
  imageViewer->SetupInteractor(renderWindowInteractor);
  imageViewer->GetRenderer()->ResetCamera();
  imageViewer->GetRenderer()->SetBackground(1,0,0);
  renderWindowInteractor->Initialize();
  renderWindowInteractor->Start();
  
  return 0 ;
}
