#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkRenderer.h>
#include <vtkJPEGReader.h>
#include <vtkImageActor.h>
#include <vtkCommand.h>
#include <vtkCallbackCommand.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkDataSetMapper.h>
#include <vtkActor.h>
#include <vtkProp3D.h>

int main ( int argc, char* argv[] )
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

  //setup image
  //vtkSmartPointer<vtkImageActor> imageActor = 
    //  vtkSmartPointer<vtkImageActor>::New();
  
  vtkSmartPointer<vtkDataSetMapper> imageMapper = 
      vtkSmartPointer<vtkDataSetMapper>::New();
  imageMapper->SetInputConnection(jPEGReader->GetOutputPort());
      
  vtkSmartPointer<vtkActor> imageActor = 
      vtkSmartPointer<vtkActor>::New();
  imageActor->SetMapper( imageMapper);

    //Create a sphere
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetCenter(0.0, 0.0, 0.0);
  sphereSource->SetRadius(300.0);
  sphereSource->Update();
 
  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> sphereMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
  vtkSmartPointer<vtkActor> sphereActor = 
      vtkSmartPointer<vtkActor>::New();
  sphereActor->SetMapper(sphereMapper);
  
  //setup renderer
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor ( imageActor );
  renderer->AddActor ( sphereActor );
  renderer->ResetCamera();

  
  //setup render window
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer ( renderer );

  //setup render window interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  
  vtkSmartPointer<vtkInteractorStyleTrackballActor> style = 
      vtkSmartPointer<vtkInteractorStyleTrackballActor>::New();
 
  renderWindowInteractor->SetInteractorStyle( style );
  
  //render and start interaction
  renderWindowInteractor->SetRenderWindow ( renderWindow );
  renderWindowInteractor->Initialize();

  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
