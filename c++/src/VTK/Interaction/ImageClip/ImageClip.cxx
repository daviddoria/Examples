#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkBorderRepresentation.h>
#include <vtkBorderWidget.h>
#include <vtkCommand.h>
#include <vtkImageActor.h>
#include <vtkImageClip.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleImage.h>
#include <vtkJPEGReader.h>
#include <vtkMath.h>
#include <vtkProperty2D.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkXMLPolyDataReader.h>
 
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
      double* lowerLeft = lowerLeftCoordinate ->GetComputedWorldValue(this->LeftRenderer);
      cout << "Lower left coordinate: " << lowerLeft[0] << ", " << lowerLeft[1] << ", " << lowerLeft[2] << endl;
 
      vtkCoordinate* upperRightCoordinate = static_cast<vtkBorderRepresentation*>(borderWidget->GetRepresentation())->GetPosition2Coordinate();
      double* upperRight = upperRightCoordinate ->GetComputedWorldValue(this->LeftRenderer);
      cout << "Upper right coordinate: " << upperRight[0] << ", " << upperRight[1] << ", " << upperRight[2] << endl;
 
      double* bounds = this->ImageActor->GetBounds();
      double xmin = bounds[0];
      double xmax = bounds[1];
      double ymin = bounds[2];
      double ymax = bounds[3];
 
      if( (lowerLeft[0] > xmin) && (upperRight[0] < xmax) && (lowerLeft[1] > ymin) && (upperRight[1] < ymax) )
        {
        //cout << "box is inside image" << endl;
        //cout << "xmin: " << xmin << " xmax: " << xmax << " ymin: " << ymin << " ymax: " << ymax << endl;
 
        this->ClipFilter->SetOutputWholeExtent(
          vtkMath::Round(lowerLeft[0]), 
          vtkMath::Round(upperRight[0]), 
          vtkMath::Round(lowerLeft[1]), 
          vtkMath::Round(upperRight[1]), 0, 1);
        }
      else
        {
        cout << "box is NOT inside image" << endl;
        }      
    }
 
    void SetLeftRenderer(vtkSmartPointer<vtkRenderer> renderer) {this->LeftRenderer = renderer;}
    void SetImageActor(vtkSmartPointer<vtkImageActor> actor)    {this->ImageActor   = actor;}
    void SetClipFilter(vtkSmartPointer<vtkImageClip> clip)      {this->ClipFilter   = clip;}
 
  private:
    vtkSmartPointer<vtkRenderer>   LeftRenderer;
    vtkSmartPointer<vtkImageActor> ImageActor;
    vtkSmartPointer<vtkImageClip>  ClipFilter;
};
 
int main(int argc, char* argv[])
{
  //parse input arguments
  if ( argc != 2 )
  {
    cout << "Required parameters: Filename" << endl;
    return EXIT_FAILURE;
  }
 
  std::string inputFilename = argv[1];
 
  //read the image
  vtkSmartPointer<vtkJPEGReader> jPEGReader = 
      vtkSmartPointer<vtkJPEGReader>::New();
 
  if( !jPEGReader->CanReadFile( inputFilename.c_str() ) )
  {
    cout << "Error: cannot read " << inputFilename << endl;
    return EXIT_FAILURE;
  }
 
  jPEGReader->SetFileName ( inputFilename.c_str() );
  jPEGReader->Update();
 
  int extent[6];
  jPEGReader->GetOutput()->GetExtent(extent);
  //xmin, xmax, ymin, ymax
  //cout << "extent: " << extent[0] << " " << extent[1] << " " << extent[2] << " " <<  extent[3] << " " <<  extent[4] << " " <<  extent[5] << endl;
 
  vtkSmartPointer<vtkImageActor> imageActor = 
      vtkSmartPointer<vtkImageActor>::New();
  imageActor->SetInput(jPEGReader->GetOutput());
 
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
 
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
 
  vtkSmartPointer<vtkInteractorStyleImage> style = 
      vtkSmartPointer<vtkInteractorStyleImage>::New();
  interactor->SetInteractorStyle( style );  
 
  vtkSmartPointer<vtkBorderWidget> borderWidget = 
      vtkSmartPointer<vtkBorderWidget>::New();
  borderWidget->SetInteractor(interactor);
  static_cast<vtkBorderRepresentation*>(borderWidget->GetRepresentation())->GetBorderProperty()->SetColor(0,1,0);
  borderWidget->SelectableOff();
 
  interactor->SetRenderWindow(renderWindow);
 
 
  //Define viewport ranges in normalized coordinates
  //(xmin, ymin, xmax, ymax)
  double leftViewport[4] = {0.0, 0.0, 0.5, 1.0};
  double rightViewport[4] = {0.5, 0.0, 1.0, 1.0};
 
  //setup both renderers
  vtkSmartPointer<vtkRenderer> leftRenderer = 
      vtkSmartPointer<vtkRenderer>::New();
  leftRenderer->SetBackground(1,0,0);
  renderWindow->AddRenderer(leftRenderer);
  leftRenderer->SetViewport(leftViewport);
 
  vtkSmartPointer<vtkRenderer> rightRenderer = 
      vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(rightRenderer);
  rightRenderer->SetViewport(rightViewport);
 
  leftRenderer->AddActor(imageActor);
 
  leftRenderer->ResetCamera();
  rightRenderer->ResetCamera();
 
  vtkSmartPointer<vtkImageClip> imageClip = 
      vtkSmartPointer<vtkImageClip>::New();
  imageClip->SetInputConnection(jPEGReader->GetOutputPort());
  imageClip->SetOutputWholeExtent(jPEGReader->GetOutput()->GetWholeExtent());
  imageClip->ClipDataOn();
 
  vtkSmartPointer<vtkImageActor> clipActor = 
      vtkSmartPointer<vtkImageActor>::New();
  clipActor->SetInput(imageClip->GetOutput());
 
  rightRenderer->AddActor(clipActor);
 
  vtkSmartPointer<vtkBorderCallback> borderCallback = 
      vtkSmartPointer<vtkBorderCallback>::New();
  borderCallback->SetLeftRenderer(leftRenderer);
  borderCallback->SetImageActor(imageActor);
  borderCallback->SetClipFilter(imageClip);
 
  borderWidget->AddObserver(vtkCommand::InteractionEvent,borderCallback);
 
  renderWindow->Render();
  borderWidget->On();
  interactor->Start();
 
  return EXIT_SUCCESS;
}