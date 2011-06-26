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
 
    virtual void Execute(vtkObject *caller, unsigned long, void*);
 
    void SetRenderer(vtkSmartPointer<vtkRenderer> renderer) {this->Renderer = renderer;}
    void SetImageActor(vtkSmartPointer<vtkImageActor> actor)    {this->ImageActor   = actor;}
 
  private:
    vtkSmartPointer<vtkRenderer>   Renderer;
    vtkSmartPointer<vtkImageActor> ImageActor;
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
   
  //shift the image center to (0,0)
  int dims[3];
  jPEGReader->GetOutput()->GetDimensions(dims);

  jPEGReader->SetDataOrigin(-dims[0]/2, -dims[1]/2, -dims[2]/2);
  jPEGReader->Update();
  
  vtkSmartPointer<vtkImageData> image = jPEGReader->GetOutput();
  
  
  //set lower left corner
  //cout << "new origin: " << -dims[0]/2 << " " << -dims[1]/2 << " " << -dims[2]/2 << endl;
  //image->SetOrigin(-dims[0]/2, -dims[1]/2, -dims[2]/2);
  //image->Update();
  
  vtkSmartPointer<vtkImageActor> imageActor = 
      vtkSmartPointer<vtkImageActor>::New();
  imageActor->SetInput(image);
 
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
 
  //setup both renderers
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  renderer->SetBackground(1,0,0);
  renderWindow->AddRenderer(renderer);
  
  renderer->AddActor(imageActor);
 
  renderer->ResetCamera();
   
  vtkSmartPointer<vtkBorderCallback> borderCallback = 
      vtkSmartPointer<vtkBorderCallback>::New();
  borderCallback->SetRenderer(renderer);
  borderCallback->SetImageActor(imageActor);
 
  borderWidget->AddObserver(vtkCommand::InteractionEvent,borderCallback);
 
  //renderWindow->Render();
  borderWidget->On();
  interactor->Start();
 
  return EXIT_SUCCESS;
}

void vtkBorderCallback::Execute(vtkObject *caller, unsigned long, void*)
{

  vtkBorderWidget *borderWidget = 
      reinterpret_cast<vtkBorderWidget*>(caller);

  //get the world coordinates of the two corners of the box
  vtkCoordinate* lowerLeftCoordinate = static_cast<vtkBorderRepresentation*>(borderWidget->GetRepresentation())->GetPositionCoordinate();
  double* lowerLeft = lowerLeftCoordinate ->GetComputedWorldValue(this->Renderer);
  cout << "Lower left coordinate: " << lowerLeft[0] << ", " << lowerLeft[1] << ", " << lowerLeft[2] << endl;

  vtkCoordinate* upperRightCoordinate = static_cast<vtkBorderRepresentation*>(borderWidget->GetRepresentation())->GetPosition2Coordinate();
  double* upperRight = upperRightCoordinate ->GetComputedWorldValue(this->Renderer);
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
      }
  else
    {
    cout << "box is NOT inside image" << endl;
    }
}