#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageDifference.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkImageActor.h>

int main(int argc, char *argv[])
{
  //create an image
  vtkSmartPointer<vtkImageCanvasSource2D> source1 = 
      vtkSmartPointer<vtkImageCanvasSource2D>::New();
  source1->SetScalarTypeToUnsignedChar();
  source1->SetNumberOfScalarComponents(3);
  source1->SetExtent(0, 100, 0, 100, 0, 0);
  source1->SetDrawColor(0,0,0,1);
  source1->FillBox(0, 100, 0, 100);
  source1->SetDrawColor(255,0,0,1);
  source1->FillBox(10, 90, 10, 90);
  source1->Update();

  //create another image
  vtkSmartPointer<vtkImageCanvasSource2D> source2 = 
      vtkSmartPointer<vtkImageCanvasSource2D>::New();
  source2->SetScalarTypeToUnsignedChar();
  source2->SetNumberOfScalarComponents(3);
  source2->SetExtent(0, 100, 0, 100, 0, 0);
  source2->SetDrawColor(0,0,0,1);
  source2->FillBox(0, 100, 0, 100);
  source2->SetDrawColor(255,0,0,1);
  source2->FillBox(20, 80, 20, 80);
  source2->Update();

  vtkSmartPointer<vtkImageDifference> differenceFilter = 
      vtkSmartPointer<vtkImageDifference>::New();
      
  differenceFilter->SetInputConnection(source1->GetOutputPort());
  differenceFilter->SetImage(source2->GetOutput());
  differenceFilter->Update();
    
  //Define viewport ranges
  //(xmin, ymin, xmax, ymax)
  double leftViewport[4] = {0.0, 0.0, 0.33, 1.0};
  double centerViewport[4] = {0.33, 0.0, 0.66, 1.0};
  double rightViewport[4] = {0.66, 0.0, 1.0, 1.0};
 
  //setup render window
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  
  //setup renderers
  vtkSmartPointer<vtkRenderer> leftRenderer = 
      vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(leftRenderer);
  leftRenderer->SetViewport(leftViewport);
  vtkSmartPointer<vtkImageActor> leftActor = 
      vtkSmartPointer<vtkImageActor>::New();
  leftActor->SetInput(source1->GetOutput());
  leftRenderer->AddActor(leftActor);
  
  vtkSmartPointer<vtkRenderer> centerRenderer = 
      vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(centerRenderer);
  centerRenderer->SetViewport(centerViewport);
   vtkSmartPointer<vtkImageActor> centerActor = 
      vtkSmartPointer<vtkImageActor>::New();
  centerActor->SetInput(source2->GetOutput());
  centerRenderer->AddActor(centerActor);
  
  vtkSmartPointer<vtkRenderer> rightRenderer = 
      vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(rightRenderer);
  rightRenderer->SetViewport(rightViewport);
    vtkSmartPointer<vtkImageActor> rightActor = 
      vtkSmartPointer<vtkImageActor>::New();
  rightActor->SetInput(differenceFilter->GetOutput());
  rightRenderer->AddActor(rightActor);
  
    //setup render window interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
 
  //render and start interaction
  renderWindowInteractor->SetRenderWindow ( renderWindow );
  renderWindowInteractor->Initialize();
 
  renderWindowInteractor->Start();
 
  return EXIT_SUCCESS;
}
