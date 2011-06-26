#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkJPEGReader.h>
#include <vtkImageActor.h>
#include <vtkCommand.h>
#include <vtkCallbackCommand.h>
#include <vtkSmartPointer.h>
#include <vtkInteractorStyleRubberBand2D.h>
#include <vtkImageActorPointPlacer.h>
#include <vtkRendererCollection.h>
#include <vtkActorCollection.h>

void SelectionChangedCallbackFunction ( vtkObject* caller, long unsigned int eventId, void* clientData, void* callData );

int main ( int argc, char* argv[] )
{
  //parse input arguments
  if ( argc != 2 )
  {
    std::cout << "Required parameters: Filename" << std::endl;
    exit ( -1 );
  }

  vtkstd::string InputFilename = argv[1];

  //read the image
  vtkSmartPointer<vtkJPEGReader> JPEGReader = vtkSmartPointer<vtkJPEGReader>::New();
  JPEGReader->SetFileName ( InputFilename.c_str() );
  JPEGReader->Update();

  //create an actor
  vtkSmartPointer<vtkImageActor> Actor = vtkSmartPointer<vtkImageActor>::New();
  Actor->SetInput ( JPEGReader->GetOutput() );

 
  //setup the SelectionChangedEvent callback
  vtkSmartPointer<vtkCallbackCommand> SelectionChangedCallback = vtkSmartPointer<vtkCallbackCommand>::New();
  SelectionChangedCallback->SetCallback ( SelectionChangedCallbackFunction );

  //setup renderer
  vtkSmartPointer<vtkRenderer> Renderer = vtkSmartPointer<vtkRenderer>::New();
  Renderer->AddActor ( Actor );
  Renderer->ResetCamera();

  //setup render window
  vtkSmartPointer<vtkRenderWindow> RenderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  RenderWindow->AddRenderer ( Renderer );

  //setup 2D interaction style
  vtkSmartPointer<vtkInteractorStyleRubberBand2D> style = vtkSmartPointer<vtkInteractorStyleRubberBand2D>::New();
  style->AddObserver ( vtkCommand::SelectionChangedEvent,SelectionChangedCallback );

  //setup render window interactor
  vtkSmartPointer<vtkRenderWindowInteractor> RenderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  RenderWindowInteractor->SetInteractorStyle ( style );

  //render and start interaction
  RenderWindowInteractor->SetRenderWindow ( RenderWindow );
  RenderWindowInteractor->Initialize();

  RenderWindowInteractor->Start();

  return 0 ;
}

void SelectionChangedCallbackFunction ( vtkObject* caller, long unsigned int eventId, void* clientData, void* callData )
{
  vtkstd::cout << "caller: " << caller->GetClassName() << vtkstd::endl;
    
  unsigned int* rect = reinterpret_cast<unsigned int*> ( callData );
  unsigned int pos1X = rect[0];
  unsigned int pos1Y = rect[1];
  unsigned int pos2X = rect[2];
  unsigned int pos2Y = rect[3];

  vtkstd::cout << "Start x: " << pos1X << " Start y: " << pos1Y << " End x: " << pos2X << " End y: " << pos2Y << vtkstd::endl;
  
  vtkInteractorStyleRubberBand2D* rb = reinterpret_cast<vtkInteractorStyleRubberBand2D*>(caller);
  
  vtkRenderer* renderer = rb->GetCurrentRenderer();
  if(renderer == NULL)
  {
    vtkstd::cout << "Invalid renderer!" << vtkstd::endl;
    return;
  }
  
  vtkstd::cout << "Getting actor..." << vtkstd::endl;
  vtkActorCollection* ActorCollection = renderer->GetActors();
  ActorCollection->InitTraversal();
  vtkActor* actor = ActorCollection->GetNextActor();
  
  if(actor == NULL)
  {
    vtkstd::cout << "Invalid actor!" << vtkstd::endl;
    return;
  }
  
  vtkImageActor* ImageActor = reinterpret_cast<vtkImageActor*>(actor);
  
  vtkstd::cout << "Setting up point placer..." << vtkstd::endl;
  
  vtkSmartPointer<vtkImageActorPointPlacer> ImageActorPointPlacer = vtkSmartPointer<vtkImageActorPointPlacer>::New();
  ImageActorPointPlacer->SetImageActor(ImageActor);
  
  double displayPos[2];
  displayPos[0] = pos1X;
  displayPos[1] = pos1Y;
  
  double worldPos[3];
  double worldOrient[9];
  
  
  vtkstd::cout << "Compute world position..." << vtkstd::endl;
  
  ImageActorPointPlacer->ComputeWorldPosition(renderer, displayPos, worldPos, worldOrient);
  
  vtkstd::cout << "World pos: " << worldPos[0] << " " << worldPos[1] << " " << worldPos[2] << vtkstd::endl;
  
}
