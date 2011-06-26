#include <vtkPolyDataMapper.h>
#include <vtkObjectFactory.h>
#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkInteractorStyleTrackballCamera.h>

// Define interaction style
class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static KeyPressInteractorStyle* New();
    vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);

    virtual void OnKeyDown()
    {
      std::cout << "KeyDown" << std::endl;
      vtkInteractorStyleTrackballCamera::OnKeyDown();
    }

    virtual void OnKeyUp()
    {
      std::cout << "KeyUp" << std::endl;
      vtkInteractorStyleTrackballCamera::OnKeyUp();
    }

    virtual void OnKeyPress()
    {
      std::cout << "KeyPress" << std::endl;
      vtkInteractorStyleTrackballCamera::OnKeyPress();
    }

    virtual void OnKeyRelease()
    {
      std::cout << "KeyRelease" << std::endl;
      vtkInteractorStyleTrackballCamera::OnKeyRelease();
    }
    /*
    virtual void OnKeyPress()
    {
      //get the keypress
      vtkRenderWindowInteractor *rwi = this->Interactor;
      std::string key = rwi->GetKeySym();

      //output the key that was pressed
      cout << "Pressed " << key << endl;

      //handle an arrow key
      if(key.compare("Up") == 0)
        {
        cout << "The up arrow was pressed." << endl;
        }
      if(key.compare("Right") == 0)
        {
        cout << "The right arrow was pressed." << endl;
        }

      //handle a "normal" key
      if(key.compare("a") == 0)
        {
        cout << "The a key was pressed." << endl;
        }

      // forward events
      vtkInteractorStyleTrackballCamera::OnKeyPress();
    }
     */
};
vtkStandardNewMacro(KeyPressInteractorStyle);

int main(int argc, char *argv[])
{

  vtkSmartPointer<vtkSphereSource> sphereSource =
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetCenter(0.0, 0.0, 0.0);
  sphereSource->SetRadius(5.0);
  sphereSource->Update();

  vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInput(sphereSource->GetOutput());

  // create an actor
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer =
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkSmartPointer<KeyPressInteractorStyle> style = vtkSmartPointer<KeyPressInteractorStyle>::New();
  renderWindowInteractor->SetInteractorStyle( style );

  renderer->AddActor(actor);
  renderer->SetBackground(1,1,1); // Background color white

  renderWindow->Render();

  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
