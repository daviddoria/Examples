#include <vtkSmartPointer.h>
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
  imageSource->SetScalarTypeToUnsignedChar();
  imageSource->SetNumberOfScalarComponents(3);
  imageSource->SetExtent(0, 300, 0, 300, 0, 0);
  imageSource->SetDrawColor(0, 0, 0);
  imageSource->FillBox(0, 300, 0, 300);
  imageSource->SetDrawColor(255, 0, 0);
  imageSource->FillTriangle(10, 100,  190, 150,  40, 250);
  imageSource->Update();
  
  //kernel
  vtkSmartPointer<vtkImageCanvasSource2D> kernelSource = 
      vtkSmartPointer<vtkImageCanvasSource2D>::New();
  kernelSource->SetScalarTypeToUnsignedChar();
  kernelSource->SetNumberOfScalarComponents(3);
  kernelSource->SetExtent(0, 30, 0, 30, 0, 0);
  kernelSource->SetDrawColor(0,0,0);
  kernelSource->FillBox(0, 30, 0, 30);
  kernelSource->SetDrawColor(255, 0,0);
  kernelSource->FillTriangle(10, 1,  25, 10,  1, 5);
  kernelSource->Update();
  
  vtkSmartPointer<vtkImageCorrelation> correlationFilter = 
      vtkSmartPointer<vtkImageCorrelation>::New();
  correlationFilter->SetInput1(imageSource->GetOutput());
  correlationFilter->SetInput2(kernelSource->GetOutput());
  correlationFilter->Update();
    
  int comp = correlationFilter->GetOutput()->GetNumberOfScalarComponents();
  cout << "comp = " << comp << endl;
  
  vtkImageData* corr = correlationFilter->GetOutput();
  int* dims = corr->GetDimensions();
  cout << "dims: " << dims[0] << " " << dims[1] << endl;
  unsigned int counter = 0;
  for (int y = 0; y < dims[1]; y++)
  {
    for (int x = 0; x < dims[0]; x++)
    {
        //double* v = static_cast<double*>(corr->GetScalarPointer(x,y,0));
      float* v = static_cast<float*>(corr->GetScalarPointer(x,y,0));
      
      if(v[0] != 0)
      {
          //cout << v[0] << endl;
        counter++;
      }

    }
    //cout << endl;
  }
  cout << counter << " pixels are nonzero." << endl;
  
  vtkSmartPointer<vtkImageCast> castFilter = 
      vtkSmartPointer<vtkImageCast>::New();
  castFilter->SetOutputScalarTypeToUnsignedChar();
  castFilter->SetInputConnection(correlationFilter->GetOutputPort());
  castFilter->Update();
  {
    vtkSmartPointer<vtkJPEGWriter> writer =
        vtkSmartPointer<vtkJPEGWriter>::New();
    writer->SetFileName("cast.jpg");
    writer->SetInputConnection(castFilter->GetOutputPort());
    writer->Write();
  }   
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
