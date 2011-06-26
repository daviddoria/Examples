#include <vtkSmartPointer.h>
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


class vtkBorderCallback : public vtkCommand
{
  public:
    vtkBorderCallback(){}
    
    static vtkBorderCallback *New()
    {
      return new vtkBorderCallback;
    }
    
    virtual void Execute(vtkObject *caller, unsigned long, void*)
    {
      
      vtkBorderWidget *borderWidget = 
          reinterpret_cast<vtkBorderWidget*>(caller);
      
      //get the world coordinates of the two corners of the box
      vtkCoordinate* lowerLeftCoordinate = static_cast<vtkBorderRepresentation*>(borderWidget->GetRepresentation())->GetPositionCoordinate();
      double* lowerLeft = new double[3];
      lowerLeft = lowerLeftCoordinate ->GetComputedWorldValue(this->LeftRenderer);
      cout << "Lower left coordinate: " << lowerLeft[0] << ", " << lowerLeft[1] << ", " << lowerLeft[2] << endl;
      
      vtkCoordinate* upperRightCoordinate = static_cast<vtkBorderRepresentation*>(borderWidget->GetRepresentation())->GetPosition2Coordinate();
      double* upperRight = new double[3];
      upperRight = upperRightCoordinate ->GetComputedWorldValue(this->LeftRenderer);
      cout << "Upper right coordinate: " << upperRight[0] << ", " << upperRight[1] << ", " << upperRight[2] << endl;
      
      double bounds[6];
      this->ImageActor->GetBounds(bounds);
      double xmin = bounds[0];
      double xmax = bounds[1];
      double ymin = bounds[2];
      double ymax = bounds[3];
      
      if( (lowerLeft[0] > xmin) && (upperRight[0] < xmax) && (lowerLeft[1] > ymin) && (upperRight[1] < ymax) )
      {
        //cout << "box is inside image" << endl;
        //cout << "xmin: " << xmin << " xmax: " << xmax << " ymin: " << ymin << " ymax: " << ymax << endl;
        vtkSmartPointer<vtkImageData> tempImage = 
            vtkSmartPointer<vtkImageData>::New();
        tempImage->DeepCopy(this->Image);
        this->ClipFilter->SetInput(tempImage);
        
        this->ClipFilter->SetOutputWholeExtent(vtkMath::Round(lowerLeft[0]), vtkMath::Round(upperRight[0]), vtkMath::Round(lowerLeft[1]), vtkMath::Round(upperRight[1]), 0, 1);
        this->ClipFilter->Update();
        this->ClipActor->SetInput(ClipFilter->GetOutput());
      }
      else
      {
        cout << "box is NOT inside image" << endl;
      }
      
    }
    
    void SetLeftRenderer(vtkSmartPointer<vtkRenderer> ren) {this->LeftRenderer = ren;}
    void SetRightRenderer(vtkSmartPointer<vtkRenderer> ren) {this->RightRenderer = ren;}
    void SetImageActor(vtkSmartPointer<vtkImageActor> im) {this->ImageActor = im;}
    void SetClipActor(vtkSmartPointer<vtkImageActor> im) {this->ClipActor = im;}
    void SetClipFilter(vtkSmartPointer<vtkImageClip> clip) {this->ClipFilter = clip;}
    void SetImage(vtkSmartPointer<vtkImageData> im) {this->Image = im;}
    
  private:
    vtkSmartPointer<vtkRenderer> LeftRenderer;
    vtkSmartPointer<vtkRenderer> RightRenderer;
    vtkSmartPointer<vtkImageActor> ImageActor;
    vtkSmartPointer<vtkImageActor> ClipActor;
    vtkSmartPointer<vtkImageClip> ClipFilter;
    vtkSmartPointer<vtkImageData> Image;
    
};

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
  
  correlationFilter->GetOutput()->SetScalarTypeToUnsignedChar();
  
  vtkSmartPointer<vtkImageActor> imageActor = 
      vtkSmartPointer<vtkImageActor>::New();
  imageActor->SetInput(imageSource->GetOutput());
  
  vtkSmartPointer<vtkImageActor> kernelActor = 
      vtkSmartPointer<vtkImageActor>::New();
  kernelActor->SetInput(kernelSource->GetOutput());
  
  vtkSmartPointer<vtkImageActor> correlationActor = 
      vtkSmartPointer<vtkImageActor>::New();
  correlationActor->SetInput(correlationFilter->GetOutput());
  
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
  
  vtkSmartPointer<vtkBorderWidget> borderWidget = 
      vtkSmartPointer<vtkBorderWidget>::New();
  borderWidget->SetInteractor(renderWindowInteractor);
  static_cast<vtkBorderRepresentation*>(borderWidget->GetRepresentation())->GetBorderProperty()->SetColor(1,0,0);
  borderWidget->SelectableOff();
  
  vtkSmartPointer<vtkBorderCallback> borderCallback = 
      vtkSmartPointer<vtkBorderCallback>::New();
  
  borderWidget->AddObserver(vtkCommand::InteractionEvent,borderCallback);
  
  // add the actors to the scene
  leftRenderer->AddActor(imageActor);
  centerRenderer->AddActor(kernelActor);
  rightRenderer->AddActor(correlationActor);
  
  leftRenderer->SetBackground(1,1,1);
  centerRenderer->SetBackground(1,1,1);
  rightRenderer->SetBackground(1,1,1);
  
  renderWindow->Render();
  renderWindowInteractor->Initialize();
  renderWindow->Render();
  borderWidget->On();
  renderWindowInteractor->Start();
  

  return EXIT_SUCCESS;
}
