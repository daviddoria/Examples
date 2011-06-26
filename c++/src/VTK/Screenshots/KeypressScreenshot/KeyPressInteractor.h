#ifndef KEYPRESSINTERACTORSTYLE_H
#define KEYPRESSINTERACTORSTYLE_H

#include "vtkWindowToImageFilter.h"
#include "vtkPNGWriter.h"
#include "vtkActor.h"
#include "vtkInteractorStyleTrackballCamera.h"

// Define interaction style
class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static KeyPressInteractorStyle* New()
    { return new KeyPressInteractorStyle();}

    void SetActor(vtkActor* actor) { mActor = actor ; }

    virtual void OnChar() 
    {
      vtkRenderWindowInteractor *rwi = this->Interactor;		
      char ch = rwi->GetKeyCode() ;

      switch (ch) 
      {
        case 's':
          vtkstd::cout << "Pressed s." << vtkstd::endl;
          vtkSmartPointer<vtkWindowToImageFilter> WindowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
          WindowToImageFilter->SetInput(rwi->GetRenderWindow());
          WindowToImageFilter->Update();
  
          vtkSmartPointer<vtkPNGWriter> Writer = vtkSmartPointer<vtkPNGWriter>::New();
          Writer->SetFileName("screenshot.png");
          Writer->SetInput(WindowToImageFilter->GetOutput());
          Writer->Write();
          
          break ;
      }
	
		// forward events
      vtkInteractorStyleTrackballCamera::OnChar();
    }
  private:
    vtkActor* mActor ;
    vtkSphereSource* mSrc ;
};

#endif