#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleImage.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

static void CreateImage1(vtkSmartPointer<vtkImageData> image);
static void CreateImage2(vtkSmartPointer<vtkImageData> image);

int main(int argc, char* argv[])
{
  // Image 1
  vtkSmartPointer<vtkImageData> image1 =
    vtkSmartPointer<vtkImageData>::New();
  CreateImage1(image1);

  vtkSmartPointer<vtkImageActor> imageActor1 =
    vtkSmartPointer<vtkImageActor>::New();
  imageActor1->SetInput(image1);

  // Image 2
  vtkSmartPointer<vtkImageData> image2 =
    vtkSmartPointer<vtkImageData>::New();
  CreateImage2(image2);

  vtkSmartPointer<vtkImageActor> imageActor2 =
    vtkSmartPointer<vtkImageActor>::New();
  imageActor2->SetInput(image2);

  vtkSmartPointer<vtkTransform> transform =
    vtkSmartPointer<vtkTransform>::New();
  transform->PostMultiply();
  transform->Translate(0,0,1.0); // red
  transform->Translate(0,0,-1.0); // white

  imageActor2->SetUserTransform(transform);

  // Visualize
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(imageActor1);
  renderer->AddActor(imageActor2);
  renderer->ResetCamera();

  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();

  renderWindowInteractor->SetInteractorStyle(style);

  renderWindowInteractor->SetRenderWindow(renderWindow);
  renderWindowInteractor->Initialize();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}

void CreateImage1(vtkSmartPointer<vtkImageData> image)
{
  // Create a white image
  image->SetDimensions(10,10,1);
  image->SetNumberOfScalarComponents(3);
  image->SetScalarTypeToUnsignedChar();
  image->AllocateScalars();

  int* dims = image->GetDimensions();

  for (int y = 0; y < dims[1]; y++)
    {
    for (int x = 0; x < dims[0]; x++)
      {
      unsigned char* pixel = static_cast<unsigned char*>(image->GetScalarPointer(x,y,0));
      pixel[0] = 255;
      pixel[1] = 255;
      pixel[2] = 255;
      }
    }
}

void CreateImage2(vtkSmartPointer<vtkImageData> image)
{
  // Create a red image
  image->SetDimensions(10,10,1);
  image->SetNumberOfScalarComponents(3);
  image->SetScalarTypeToUnsignedChar();
  image->AllocateScalars();

  int* dims = image->GetDimensions();

  for (int y = 0; y < dims[1]; y++)
    {
    for (int x = 0; x < dims[0]; x++)
      {
      unsigned char* pixel = static_cast<unsigned char*>(image->GetScalarPointer(x,y,0));
      pixel[0] = 255;
      pixel[1] = 0;
      pixel[2] = 0;
      }
    }
}