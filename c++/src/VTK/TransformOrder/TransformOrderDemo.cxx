#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkAxesActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

int main(int argc, char *argv[])
{
  cout << "Correct" << endl << "----------" << endl;
  	
  //create the transformation
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->PostMultiply(); //this is the key line
  transform->RotateZ(90.0);
  transform->Translate(2.0, 0.0, 0.0);
  	
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

  vtkSmartPointer<vtkAxesActor> defaultAxes =
      vtkSmartPointer<vtkAxesActor>::New();
  defaultAxes->SetXAxisLabelText("Xo");
  defaultAxes->SetYAxisLabelText("Yo");
  defaultAxes->SetZAxisLabelText("Zo");
  
  vtkSmartPointer<vtkAxesActor> axes =
      vtkSmartPointer<vtkAxesActor>::New();

  // The axes are positioned with a user transform
  axes->SetUserTransform(transform);
 
  renderer->AddActor(axes);
  renderer->AddActor(defaultAxes);
 
  renderer->ResetCamera();
  renderWindow->Render();
 
  // begin mouse interaction
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
