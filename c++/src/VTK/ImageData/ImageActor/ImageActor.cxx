#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkJPEGReader.h>
#include <vtkImageActor.h>
 
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
 
  //create an actor
  vtkSmartPointer<vtkImageActor> actor = 
      vtkSmartPointer<vtkImageActor>::New();
  actor->SetInput ( jPEGReader->GetOutput() );
 
  //setup renderer
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor ( actor );
  renderer->ResetCamera();
 
  //setup render window
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer ( renderer );
 
  //setup render window interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();

#if 1
      vtkSmartPointer<vtkPoints> cornerPoints = 
          vtkSmartPointer<vtkPoints>::New();
      cornerPoints->InsertNextPoint(0,0,0);
      cornerPoints->InsertNextPoint(512,512,0);
      vtkSmartPointer<vtkPolyData> cornerVertices = 
          vtkSmartPointer<vtkPolyData>::New();
      cornerVertices->SetPoints(cornerPoints);
      
      vtkSmartPointer<vtkPolyDataMapper> cornerMapper = 
        vtkSmartPointer<vtkPolyDataMapper>::New();
      cornerMapper->ScalarVisibilityOff();
      cornerMapper->SetInput(cornerVertices);
      vtkSmartPointer<vtkActor> cornerActor = 
          vtkSmartPointer<vtkActor>::New();
      cornerActor->SetMapper(cornerMapper);
      cornerActor->GetProperty()->SetColor(0.0, 1.0, 0.0);
      cornerActor->GetProperty()->SetPointSize(50);
      renderer->AddActor(cornerActor);
      
      
#else
      vtkSmartPointer<vtkSphereSource> sphereSource = 
          vtkSmartPointer<vtkSphereSource>::New();
      sphereSource->SetRadius(20);
      sphereSource->SetCenter(0,0,0);
      sphereSource->Update();
      vtkSmartPointer<vtkPolyDataMapper> centerMapper = 
        vtkSmartPointer<vtkPolyDataMapper>::New();
      centerMapper->SetInputConnection(sphereSource->GetOutputPort());
      vtkSmartPointer<vtkActor> centerActor = 
          vtkSmartPointer<vtkActor>::New();
      centerActor->SetMapper(centerMapper);
      renderer->AddActor(centerActor);
      
      vtkSmartPointer<vtkSphereSource> sphereSource2 = 
          vtkSmartPointer<vtkSphereSource>::New();
      sphereSource2->SetRadius(20);
      sphereSource2->SetCenter(512, 512, 0);
      sphereSource2->Update();
      vtkSmartPointer<vtkPolyDataMapper> cornerMapper = 
        vtkSmartPointer<vtkPolyDataMapper>::New();
      cornerMapper->SetInputConnection(sphereSource2->GetOutputPort());
      vtkSmartPointer<vtkActor> cornerActor = 
          vtkSmartPointer<vtkActor>::New();
      cornerActor->SetMapper(cornerMapper);
      renderer->AddActor(cornerActor);
#endif
 
  //render and start interaction
  renderWindowInteractor->SetRenderWindow ( renderWindow );
  renderWindowInteractor->Initialize();
 
  renderWindowInteractor->Start();
 
  return EXIT_SUCCESS;
}