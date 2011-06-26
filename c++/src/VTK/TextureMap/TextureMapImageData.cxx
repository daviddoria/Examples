#include <vtkImageData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTextureMapToPlane.h>
#include <vtkPlaneSource.h>
#include <vtkTexture.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkImageCanvasSource2D.h>

int main ( int argc, char *argv[] )
{
  //create an image
  vtkSmartPointer<vtkImageCanvasSource2D> imageSource = 
      vtkSmartPointer<vtkImageCanvasSource2D>::New();
  //imageSource->SetScalarTypeToDouble();
  imageSource->SetScalarTypeToUnsignedChar();
  imageSource->SetExtent(0, 20, 0, 20, 0, 0);
  imageSource->SetNumberOfScalarComponents(3);
  imageSource->FillBox(0, 20, 0, 20);
  imageSource->Update();
  
  //create a plane
  vtkSmartPointer<vtkPlaneSource> plane = 
      vtkSmartPointer<vtkPlaneSource>::New();
  plane->SetCenter(0.0, 0.0, 0.0);
  plane->SetNormal(0.0, 0.0, 1.0);
  
  //apply the texture
  vtkSmartPointer<vtkTexture> texture = 
      vtkSmartPointer<vtkTexture>::New();
  cout << "map through LUT?" << texture->GetMapColorScalarsThroughLookupTable() << endl;
  texture->SetInputConnection(imageSource->GetOutputPort());

  vtkSmartPointer<vtkTextureMapToPlane> texturePlane =
      vtkSmartPointer<vtkTextureMapToPlane>::New();
  texturePlane->SetInput(plane->GetOutput());
  
  vtkSmartPointer<vtkPolyDataMapper> planeMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  planeMapper->SetInputConnection(texturePlane->GetOutputPort());

  vtkSmartPointer<vtkActor> texturedPlane = 
      vtkSmartPointer<vtkActor>::New();
  texturedPlane->SetMapper(planeMapper);
  texturedPlane->SetTexture(texture);

  // visualize the textured plane
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(texturedPlane);
  renderer->SetBackground(1,1,1); // Background color white
  renderer->ResetCamera();
  
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
  
  renderWindow->Render();
  
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
