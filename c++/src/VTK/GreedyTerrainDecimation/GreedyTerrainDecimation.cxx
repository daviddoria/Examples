#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkMath.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkImageData.h>
#include <vtkGreedyTerrainDecimation.h>

int main(int argc, char *argv[])
{
  // Create a sphere
  vtkSmartPointer<vtkImageData> image = 
    vtkSmartPointer<vtkImageData>::New();
  image->SetDimensions(3,3,1);
  image->SetNumberOfScalarComponents(1);
  image->SetScalarTypeToUnsignedChar();
  
  int dims[3];
  image->GetDimensions(dims);
  for(unsigned int i = 0; i < dims[0]; i++)
    {
    for(unsigned int j = 0; j < dims[1]; j++)
      {
      unsigned char* pixel = static_cast<unsigned char*>(image->GetScalarPointer(i,j,0));
      //pixel[0] = round(vtkMath::Random(0, 255));
      pixel[0] = round(vtkMath::Random(0, 1));
      }
    }
  
  vtkSmartPointer<vtkGreedyTerrainDecimation> decimation =
    vtkSmartPointer<vtkGreedyTerrainDecimation>::New();
  decimation->SetInputConnection(image->GetProducerPort());
  decimation->Update();

  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(decimation->GetOutputPort());
  
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetInterpolationToFlat();
  actor->GetProperty()->EdgeVisibilityOn();
  actor->GetProperty()->SetEdgeColor(1,0,0);

  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(actor);
  renderer->SetBackground(1,1,1); // Background color white

  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}