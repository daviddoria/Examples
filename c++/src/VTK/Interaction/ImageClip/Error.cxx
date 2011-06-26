#include <vtkSmartPointer.h>
#include <vtkMath.h>
#include <vtkImageData.h>
#include <vtkImageClip.h>
#include <vtkCommand.h>
#include <vtkJPEGReader.h>
#include <vtkImageActor.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkSphereSource.h>
#include <vtkBorderWidget.h>
#include <vtkBorderRepresentation.h>
#include <vtkCubeSource.h>
#include <vtkProperty2D.h>

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
  jPEGReader->SetFileName ( inputFilename.c_str() );
  jPEGReader->Update();

  int extent[6];
  jPEGReader->GetOutput()->GetExtent(extent);
  cout << "extent: " << extent[0] << " " << extent[1] << " " << extent[2] << " " <<  extent[3] << " " <<  extent[4] << " " <<  extent[5] << endl;
      //xmin, xmax, ymin, ymax
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
  
  //setup sphere
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  vtkSmartPointer<vtkPolyDataMapper> sphereMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
  vtkSmartPointer<vtkActor> sphereActor = 
      vtkSmartPointer<vtkActor>::New();
  sphereActor->SetMapper(sphereMapper);
  
  //setup cube
  vtkSmartPointer<vtkCubeSource> cubeSource = 
      vtkSmartPointer<vtkCubeSource>::New();
  cubeSource->Update();
  vtkSmartPointer<vtkPolyDataMapper> cubeMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  cubeMapper->SetInputConnection(cubeSource->GetOutputPort());
  vtkSmartPointer<vtkActor> cubeActor = 
      vtkSmartPointer<vtkActor>::New();
  cubeActor->SetMapper(cubeMapper);

  
  //Define viewport ranges
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
  //imageClip->SetInputConnection(jPEGReader->GetOutputPort());
  imageClip->SetInput(jPEGReader->GetOutput());
  //imageClip->SetOutputWholeExtent(jPEGReader->GetOutput()->GetWholeExtent());
  imageClip->ClipDataOn();
  //imageClip->Update();
  
  vtkSmartPointer<vtkImageActor> clipActor = 
      vtkSmartPointer<vtkImageActor>::New();
  clipActor->SetInput(imageClip->GetOutput());
  
  rightRenderer->AddActor(clipActor);
  
  vtkSmartPointer<vtkBorderCallback> borderCallback = 
      vtkSmartPointer<vtkBorderCallback>::New();
  borderCallback->SetLeftRenderer(leftRenderer);
  borderCallback->SetRightRenderer(rightRenderer);
  borderCallback->SetImageActor(imageActor);
  borderCallback->SetClipActor(clipActor);
  borderCallback->SetClipFilter(imageClip);
  borderCallback->SetImage(jPEGReader->GetOutput());
  
  borderWidget->AddObserver(vtkCommand::InteractionEvent,borderCallback);
  
  renderWindow->Render();
  borderWidget->On();
  interactor->Start();
  
  return EXIT_SUCCESS;
}