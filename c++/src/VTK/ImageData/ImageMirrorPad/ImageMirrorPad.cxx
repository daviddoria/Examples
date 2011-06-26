#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageMirrorPad.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkImageActor.h>

int main(int argc, char *argv[])
{
  //create an image
  vtkSmartPointer<vtkImageCanvasSource2D> source = 
      vtkSmartPointer<vtkImageCanvasSource2D>::New();
  source->SetExtent(0, 20, 0, 20, 0, 0);
  source->SetScalarTypeToUnsignedChar();
  source->SetDrawColor(255.0, 0.0, 0.0, 0.5);
  source->DrawCircle(10, 10, 5);
  source->Update();
  
  vtkSmartPointer<vtkImageMirrorPad> mirrorPadFilter = 
      vtkSmartPointer<vtkImageMirrorPad>::New();
  mirrorPadFilter->SetInputConnection(source->GetOutputPort());
  mirrorPadFilter->SetOutputWholeExtent(-10, 30, -10, 30, 0, 0);
  mirrorPadFilter->Update();
  
    //create an actor
  vtkSmartPointer<vtkImageActor> actor = 
      vtkSmartPointer<vtkImageActor>::New();
  actor->SetInput(mirrorPadFilter->GetOutput());
 
  //setup renderer
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(actor);
  renderer->ResetCamera();
 
  //setup render window
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer ( renderer );
 
  //setup render window interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
 
  //render and start interaction
  renderWindowInteractor->SetRenderWindow ( renderWindow );
  renderWindowInteractor->Initialize();
 
  renderWindowInteractor->Start();
 
  
  return EXIT_SUCCESS;
}
