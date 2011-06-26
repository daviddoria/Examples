#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkJPEGReader.h>
#include <vtkImageActor.h>
#include <vtkCommand.h>
#include <vtkCallbackCommand.h>
#include <vtkSmartPointer.h>
#include <vtkPointPicker.h>

#include "VtkObserverMouseClick.h"

#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkCamera.h"
#include "vtkImageData.h"
#include "vtkImageViewer2.h"


int main ( int argc, char* argv[] )
{
  //parse input arguments
  if ( argc != 2 )
  {
    std::cout << "Required parameters: Filename" << std::endl << "Exiting..." << std::endl;
    exit ( -1 );
  }

  vtkstd::cout << "Select Pixel:" << vtkstd::endl;
  
  vtkstd::string InputFilename = argv[1];

  //read the image
  vtkSmartPointer<vtkJPEGReader> JPEGReader = vtkSmartPointer<vtkJPEGReader>::New();
  JPEGReader->SetFileName ( InputFilename.c_str() );
  JPEGReader->Update();
  
  vtkstd::cout << "Image has: " << JPEGReader->GetOutput()->GetNumberOfScalarComponents() << " components." << vtkstd::endl;

  vtkSmartPointer<vtkRenderWindowInteractor> RenderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();

  vtkSmartPointer<vtkImageViewer2> ImageViewer = vtkSmartPointer<vtkImageViewer2>::New();
  ImageViewer->SetInput(JPEGReader->GetOutput());
  ImageViewer->SetupInteractor(RenderWindowInteractor);
  ImageViewer->GetRenderer()->ResetCamera();
  ImageViewer->GetRenderer()->SetBackground(1,0,0);
  
  vtkSmartPointer<vtkPointPicker> Picker = vtkSmartPointer<vtkPointPicker>::New();
  Picker->SetTolerance(0.0);
  RenderWindowInteractor->SetPicker(Picker);
  
  //vtkSmartPointer<VtkObserverMouseClick> observeMouseClick = vtkSmartPointer<VtkObserverMouseClick>::New(ImageViewer, RenderWindowInteractor, Picker);
  VtkObserverMouseClick* observeMouseClick = VtkObserverMouseClick::New(ImageViewer, RenderWindowInteractor, Picker);
  RenderWindowInteractor->AddObserver(vtkCommand::LeftButtonPressEvent, observeMouseClick);
  
  RenderWindowInteractor->Initialize();
  RenderWindowInteractor->Start();
  
  return 0 ;
}
