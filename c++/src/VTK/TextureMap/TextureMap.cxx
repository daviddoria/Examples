#include <vtkImageData.h>
#include <vtkJPEGReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTextureMapToPlane.h>
#include <vtkPlaneSource.h>
#include <vtkTexture.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

int main ( int argc, char *argv[] )
{
  //parse command line arguments
  if ( argc != 2 )
    {
    cout << "Required parameters: Filename" << endl;
    return EXIT_FAILURE;
    }

  vtkstd::string inputFilename = argv[1];

  //read the image which will be the texture
  vtkSmartPointer<vtkJPEGReader> jpegReader = 
      vtkSmartPointer<vtkJPEGReader>::New();
  jpegReader->SetFileName(inputFilename.c_str() );
  jpegReader->Update();
  
  //create a plane
  vtkSmartPointer<vtkPlaneSource> planeSource = 
      vtkSmartPointer<vtkPlaneSource>::New();
  planeSource->SetCenter(0.0, 0.0, 0.0);
  planeSource->SetNormal(0.0, 0.0, 1.0);
  planeSource->Update();
  
  //apply the texture
  vtkSmartPointer<vtkTexture> texture = 
      vtkSmartPointer<vtkTexture>::New();
  texture->SetInput(jpegReader->GetOutput());

  vtkSmartPointer<vtkTextureMapToPlane> texturePlane =
      vtkSmartPointer<vtkTextureMapToPlane>::New();
  texturePlane->SetInputConnection(planeSource->GetOutputPort());
  
  vtkSmartPointer<vtkPolyDataMapper> planeMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  planeMapper->SetInputConnection(texturePlane->GetOutputPort());

  vtkSmartPointer<vtkActor> texturedPlane = 
      vtkSmartPointer<vtkActor>::New();
  texturedPlane->SetMapper(planeMapper);
  texturedPlane->SetTexture(texture);

  // visualize the textured plane
  vtkSmartPointer<vtkRenderer> Renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  Renderer->AddActor(texturedPlane);
  Renderer->SetBackground(1,1,1); // Background color white
  Renderer->ResetCamera();
  
  vtkSmartPointer<vtkRenderWindow> RenderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  RenderWindow->AddRenderer(Renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> RenderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  RenderWindowInteractor->SetRenderWindow(RenderWindow);
  
  RenderWindow->Render();
  
  RenderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
