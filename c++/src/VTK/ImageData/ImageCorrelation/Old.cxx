#include <vtkSmartPointer.h>
#include <vtkImageMathematics.h>
#include <vtkJPEGWriter.h>
#include <vtkImageCast.h>
#include <vtkMath.h>
#include <vtkProperty2D.h>
#include <vtkImageClip.h>
#include <vtkCoordinate.h>
#include <vtkImageData.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageCorrelation.h>
#include <vtkImageViewer.h>
#include <vtkInteractorStyleImage.h>
#include <vtkImageActor.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkBorderWidget.h>
#include <vtkBorderRepresentation.h>
#include <vtkCommand.h>

int main(int argc, char *argv[])
{
  //create an image
  vtkSmartPointer<vtkImageCanvasSource2D> imageSource = 
      vtkSmartPointer<vtkImageCanvasSource2D>::New();
  imageSource->SetNumberOfScalarComponents(3);
  imageSource->SetScalarTypeToUnsignedChar();
  imageSource->SetExtent(0, 300, 0, 300, 0, 0);
  imageSource->SetDrawColor(0, 0, 0);
  imageSource->FillBox(0, 300, 0, 300);
  imageSource->SetDrawColor(255, 0, 0); //red
  imageSource->FillTriangle(10, 100,  190, 150,  40, 250);
  imageSource->Update();
  
  //write the image
{
  vtkSmartPointer<vtkJPEGWriter> writer =
      vtkSmartPointer<vtkJPEGWriter>::New();
  writer->SetFileName("image.jpg");
  writer->SetInputConnection(imageSource->GetOutputPort());
  writer->Write();
}   
  //create a kernel
  vtkSmartPointer<vtkImageCanvasSource2D> kernelSource = 
      vtkSmartPointer<vtkImageCanvasSource2D>::New();
  kernelSource->SetNumberOfScalarComponents(3);
  kernelSource->SetScalarTypeToUnsignedChar();
  kernelSource->SetExtent(0, 30, 0, 30, 0, 0);
  kernelSource->SetDrawColor(0, 0, 0);
  kernelSource->FillBox(0, 30, 0, 30);
  kernelSource->SetDrawColor(255, 0, 0); //red
  kernelSource->FillTriangle(10, 1,  25, 10,  1, 5);
  kernelSource->Update();
  
  //write the kernel
{
  vtkSmartPointer<vtkJPEGWriter> writer =
      vtkSmartPointer<vtkJPEGWriter>::New();
  writer->SetFileName("kernel.jpg");
  writer->SetInputConnection(kernelSource->GetOutputPort());
  writer->Write();
}   

  //compute the correlation
  vtkSmartPointer<vtkImageCorrelation> correlationFilter = 
      vtkSmartPointer<vtkImageCorrelation>::New();
  correlationFilter->SetInput1(imageSource->GetOutput());
  correlationFilter->SetInput2(kernelSource->GetOutput());
  correlationFilter->Update();
    
  //at this point, corr pixels are doubles
  vtkImageData* corr = correlationFilter->GetOutput();

  //multiply every pixel by 255
  vtkSmartPointer<vtkImageMathematics> imageMath = 
      vtkSmartPointer<vtkImageMathematics>::New();
  imageMath->SetOperationToMultiplyByK();
  imageMath->SetConstantK(255.0);
  imageMath->SetInput1(corr);
  imageMath->Update();
  
  //conver the doubles to unsigned chars for writing
  vtkSmartPointer<vtkImageCast> castFilter = 
      vtkSmartPointer<vtkImageCast>::New();
  castFilter->SetOutputScalarTypeToUnsignedChar();
  castFilter->SetInputConnection(imageMath->GetOutputPort());
  castFilter->Update();
  
  vtkSmartPointer<vtkJPEGWriter> writer =
      vtkSmartPointer<vtkJPEGWriter>::New();
  writer->SetFileName("correlation.jpg");
  writer->SetInputConnection(castFilter->GetOutputPort());
  writer->Write();
  

  return EXIT_SUCCESS;
}

#if 0
  vtkSmartPointer<vtkImageActor> imageActor = 
      vtkSmartPointer<vtkImageActor>::New();
  imageActor->SetInput(imageSource->GetOutput());
  
  vtkSmartPointer<vtkImageActor> kernelActor = 
      vtkSmartPointer<vtkImageActor>::New();
  kernelActor->SetInput(kernelSource->GetOutput());
  
  vtkSmartPointer<vtkImageActor> correlationActor = 
      vtkSmartPointer<vtkImageActor>::New();
  correlationActor->SetInput(castFilter->GetOutput());
  
  //Define viewport ranges
  //(xmin, ymin, xmax, ymax)
  double leftViewport[4] = {0.0, 0.0, 0.33, 1.0};
  double centerViewport[4] = {0.33, 0.0, .66, 1.0};
  double rightViewport[4] = {0.66, 0.0, 1.0, 1.0};
  
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  
  //setup renderers
  vtkSmartPointer<vtkRenderer> leftRenderer = 
      vtkSmartPointer<vtkRenderer>::New();
  leftRenderer->SetBackground(1,0,0);
  renderWindow->AddRenderer(leftRenderer);
  leftRenderer->SetViewport(leftViewport);
  
  vtkSmartPointer<vtkRenderer> centerRenderer = 
      vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(centerRenderer);
  centerRenderer->SetViewport(centerViewport);
  
  vtkSmartPointer<vtkRenderer> rightRenderer = 
      vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(rightRenderer);
  rightRenderer->SetViewport(rightViewport);


  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
  
  vtkSmartPointer<vtkInteractorStyleImage> style = 
      vtkSmartPointer<vtkInteractorStyleImage>::New();
 
  renderWindowInteractor->SetInteractorStyle( style );
  
  // add the actors to the scene
  leftRenderer->AddActor(imageActor);
  centerRenderer->AddActor(kernelActor);
  rightRenderer->AddActor(correlationActor);
  
  leftRenderer->ResetCamera();
  centerRenderer->ResetCamera();
  rightRenderer->ResetCamera();
  
  leftRenderer->SetBackground(1,1,1);
  centerRenderer->SetBackground(1,1,1);
  rightRenderer->SetBackground(1,1,1);
  
  renderWindow->Render();
  renderWindowInteractor->Initialize();
  renderWindow->Render();
  renderWindowInteractor->Start();


  return EXIT_SUCCESS;
}
#endif  