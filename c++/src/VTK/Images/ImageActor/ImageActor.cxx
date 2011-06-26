#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkImageActor.h>
#include <vtkObjectFactory.h>

void CreateGrayscaleImage(vtkImageData*);
void CreateColorImage(vtkImageData*);

class KeyPressInteractorStyle : public vtkInteractorStyleImage
{
  public:
    static KeyPressInteractorStyle* New();
    vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleImage);

    virtual void OnKeyPress()
    {
      // Get the keypress
      std::string key = this->Interactor->GetKeySym();

      if(key.compare("s") == 0) // 's' for 's'witch
        {
        if(image == grayscaleImage)
          {
          image = colorImage;
          actor->SetInput(colorImage);
          std::cout << "Color" << std::endl;
          }
        else
          {
          image = grayscaleImage;
          actor->SetInput(grayscaleImage);
          std::cout << "Grayscale" << std::endl;
          }
        
        //this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);
        }

      // Forward events
      vtkInteractorStyleTrackballCamera::OnKeyPress();
    }

  vtkImageData* grayscaleImage;
  vtkImageData* colorImage;
  vtkImageData* image;
  
  vtkImageActor* actor;

};
vtkStandardNewMacro(KeyPressInteractorStyle);

int main(int, char *[])
{
  // Create a grayscale image
  vtkSmartPointer<vtkImageData> grayscaleImage =
    vtkSmartPointer<vtkImageData>::New();
  CreateGrayscaleImage(grayscaleImage);

  vtkSmartPointer<vtkImageData> colorImage =
    vtkSmartPointer<vtkImageData>::New();
  CreateColorImage(colorImage);
  
  vtkSmartPointer<vtkImageActor> actor =
    vtkSmartPointer<vtkImageActor>::New();
  
  // Setup renderers
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(actor);
  renderer->ResetCamera();

  // Setup render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(300, 300);
  renderWindow->AddRenderer(renderer);

  // Setup render window interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
    
  vtkSmartPointer<KeyPressInteractorStyle> style =
    vtkSmartPointer<KeyPressInteractorStyle>::New();
  
  style->actor = actor;
  style->grayscaleImage = grayscaleImage;
  style->colorImage = colorImage;

//#define DemonstrateBug
  
#ifdef DemonstrateBug
  // Demonstrates the bug - pressing 's' to toggle between grayscale and color does not work, both images are grayscale
  style->image = grayscaleImage;
  actor->SetInput(grayscaleImage);
#else
  // Works properly - pressing 's' to toggle between grayscale and color works properly
  style->image = colorImage;
  actor->SetInput(colorImage);
#endif
  
  renderWindowInteractor->SetInteractorStyle(style);

  // Render and start interaction
  renderWindowInteractor->SetRenderWindow(renderWindow);
  renderWindowInteractor->Initialize();

  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}


void CreateGrayscaleImage(vtkImageData* image)
{
  image->SetNumberOfScalarComponents(1);
  image->SetScalarTypeToUnsignedChar();
  image->SetDimensions(10, 10, 1);

  image->AllocateScalars();

  for(unsigned int x = 0; x < 10; x++)
    {
    for(unsigned int y = 0; y < 10; y++)
      {
      unsigned char* pixel = static_cast<unsigned char*>(image->GetScalarPointer(x,y,0));
      pixel[0] = 50;
      }
    }

}

void CreateColorImage(vtkImageData* image)
{
  image->SetNumberOfScalarComponents(3);
  image->SetScalarTypeToUnsignedChar();
  image->SetDimensions(10, 10, 1);

  image->AllocateScalars();

  for(unsigned int x = 0; x < 10; x++)
    {
    for(unsigned int y = 0; y < 10; y++)
      {
      unsigned char* pixel = static_cast<unsigned char*>(image->GetScalarPointer(x,y,0));
      pixel[0] = 255;
      pixel[1] = 0;
      pixel[2] = 255;
      }
    }
}
