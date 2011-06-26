#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkImageClip.h>
#include <vtkProperty2D.h>
#include <vtkImageActor.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkJPEGReader.h>
#include <vtkBorderWidget.h>
#include <vtkBorderRepresentation.h>
#include <vtkCommand.h>
#include <vtkPropPicker.h>
#include <vtkInteractorStyleImage.h>
#include <vtkCoordinate.h>

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
        cout << "box is inside image" << endl;
      }
      else
      {
        cout << "box is NOT inside image" << endl;
      }
      
    }
    
    void SetLeftRenderer(vtkSmartPointer<vtkRenderer> ren) {this->LeftRenderer = ren;}
    void SetRightRenderer(vtkSmartPointer<vtkRenderer> ren) {this->RightRenderer = ren;}
    void SetImageActor(vtkSmartPointer<vtkImageActor> im) {this->ImageActor = im;}
    
  private:
    vtkSmartPointer<vtkRenderer> LeftRenderer;
    vtkSmartPointer<vtkRenderer> RightRenderer;
    vtkSmartPointer<vtkImageActor> ImageActor;
    
};

int main (int argc, char *argv[])
{
  //parse input arguments
  if ( argc != 2 )
  {
    cout << "Required parameters: Filename" << endl;
    return EXIT_FAILURE;
  }
 
  std::string InputFilename = argv[1];
 
  //read the image
  vtkSmartPointer<vtkJPEGReader> jPEGReader = 
      vtkSmartPointer<vtkJPEGReader>::New();
  jPEGReader->SetFileName ( InputFilename.c_str() );
  jPEGReader->Update();

  vtkSmartPointer<vtkImageActor> actor = 
      vtkSmartPointer<vtkImageActor>::New();
  actor->SetInput(jPEGReader->GetOutput());
  

  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();

  
  //Define viewport ranges
  double leftViewport[4] = {0.0, 0.5, 0.0, 1.0};
  double rightViewport[4] = {0.5, 1.0, 0.0, 1.0};
  
  //setup both renderers
  vtkSmartPointer<vtkRenderer> leftRenderer = 
      vtkSmartPointer<vtkRenderer>::New();
  leftRenderer->SetViewport(leftViewport);
  
  vtkSmartPointer<vtkRenderer> rightRenderer = 
      vtkSmartPointer<vtkRenderer>::New();
  rightRenderer->SetViewport(rightViewport);
  
  //add both renderers to the window
  renderWindow->AddRenderer(leftRenderer);
  renderWindow->AddRenderer(rightRenderer);
  
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
  
  vtkSmartPointer<vtkImageClip> imageClip = 
      vtkSmartPointer<vtkImageClip>::New();
  imageClip->SetInputConnection(jPEGReader->GetOutputPort());
  //clip->SetOutputWholeExtent(0,66,0,66,30,37);
  imageClip->SetOutputWholeExtent(jPEGReader->GetOutput()->GetWholeExtent());
  imageClip->ClipDataOn();
  imageClip->Update();
  
  vtkSmartPointer<vtkBorderCallback> borderCallback = 
      vtkSmartPointer<vtkBorderCallback>::New();
  borderCallback->SetLeftRenderer(leftRenderer);
  borderCallback->SetRightRenderer(rightRenderer);
  borderCallback->SetImageActor(actor);
  
  borderWidget->AddObserver(vtkCommand::InteractionEvent,borderCallback);
  
  // add the actors to the scene
  leftRenderer->AddActor(actor);
  leftRenderer->SetBackground(1,1,1);
  rightRenderer->SetBackground(1,1,1);
  
  renderWindow->Render();
  renderWindowInteractor->Initialize();
  renderWindow->Render();
  borderWidget->On();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
