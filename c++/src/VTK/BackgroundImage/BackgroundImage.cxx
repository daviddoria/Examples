#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkJPEGReader.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
 
int main(int argc, char *argv[])
{
   //Verify input arguments
  if ( argc != 2 )
    {
    cout << "Required parameters: (jpeg) Filename" << endl << "Exiting..." << endl;
    return EXIT_FAILURE;
    }

  //Read the image
  vtkSmartPointer<vtkJPEGReader> jpegReader = vtkSmartPointer<vtkJPEGReader>::New();
  if( !jpegReader->CanReadFile( argv[1] ) )
    {
    cout << "Error reading file " << argv[1] << endl << "Exiting..." << endl;
    return EXIT_FAILURE;
    }
  jpegReader->SetFileName ( argv[1] );

  // Create an image actor to display the image
  vtkSmartPointer<vtkImageActor> imageActor =
      vtkSmartPointer<vtkImageActor>::New();
  imageActor->SetInput(jpegReader->GetOutput());

  // Create a renderer to display the image in the background
  vtkSmartPointer<vtkRenderer> backgroundRenderer = 
      vtkSmartPointer<vtkRenderer>::New();

  //Create a sphere
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetRadius(50);
  sphereSource->Update();
 
  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> sphereMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
 
  vtkSmartPointer<vtkActor> sphereActor = 
      vtkSmartPointer<vtkActor>::New();
  sphereActor->SetMapper(sphereMapper);
 
  vtkSmartPointer<vtkRenderer> sceneRenderer = 
      vtkSmartPointer<vtkRenderer>::New();
 
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
 
  // Set up the render window and renderers such that there is
  // a background layer and a foreground layer
  backgroundRenderer->SetLayer(0);
  backgroundRenderer->InteractiveOff();
  sceneRenderer->SetLayer(1);
  renderWindow->SetNumberOfLayers(2);
  renderWindow->AddRenderer(backgroundRenderer);
  renderWindow->AddRenderer(sceneRenderer);
 
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
 
  //Add actors to the renderers
  sceneRenderer->AddActor(sphereActor);
  backgroundRenderer->AddActor(imageActor);
 
  //Render once to figure out where the background camera will be
  renderWindow->Render();

  //Set up the background camera to fill the renderer with the image
  double origin[3];
  double spacing[3];
  int extent[6];
  jpegReader->GetOutput()->GetOrigin( origin );
  jpegReader->GetOutput()->GetSpacing( spacing );
  jpegReader->GetOutput()->GetExtent( extent );

  vtkCamera* camera = backgroundRenderer->GetActiveCamera();
  camera->ParallelProjectionOn();

  double xc = origin[0] + 0.5*(extent[0] + extent[1])*spacing[0];
  double yc = origin[1] + 0.5*(extent[2] + extent[3])*spacing[1];
  double xd = (extent[1] - extent[0] + 1)*spacing[0];
  double yd = (extent[3] - extent[2] + 1)*spacing[1];
  double d = camera->GetDistance();
  camera->SetParallelScale(0.5*yd);
  camera->SetFocalPoint(xc,yc,0.0);
  camera->SetPosition(xc,yc,d);

  //render again to set the correct view
  renderWindow->Render();
  
  //Interact with the window
  renderWindowInteractor->Start();
 
  return EXIT_SUCCESS;
}
