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
    
  interactor->SetRenderWindow(renderWindow);
    
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
      
  //add a sphere to the left and a cube to the right
  //leftRenderer->AddActor(sphereActor);
  leftRenderer->AddActor(imageActor);
    
  leftRenderer->ResetCamera();
  rightRenderer->ResetCamera();
  
  vtkSmartPointer<vtkImageClip> imageClip = 
      vtkSmartPointer<vtkImageClip>::New();
  imageClip->SetInputConnection(jPEGReader->GetOutputPort());
  imageClip->SetOutputWholeExtent(100, 500, 100, 500, 1, 1);
  imageClip->ClipDataOn();
  imageClip->Update();
  
  vtkSmartPointer<vtkImageActor> clipActor = 
      vtkSmartPointer<vtkImageActor>::New();
  clipActor->SetInput(imageClip->GetOutput());
  
    //rightRenderer->AddActor(cubeActor);
  rightRenderer->AddActor(clipActor);
  
  renderWindow->Render();
  interactor->Start();
  
  return EXIT_SUCCESS;
}