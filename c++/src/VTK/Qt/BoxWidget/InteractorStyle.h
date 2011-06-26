#ifndef INTERACTORSTYLE_H
#define INTERACTORSTYLE_H

#include <vtkSmartPointer.h>
#include <vtkInteractorStyleTrackballCamera.h>

class vtkBoxWidget2;

class InteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
  static InteractorStyle* New();
  vtkTypeMacro(InteractorStyle, vtkInteractorStyleTrackballCamera);
  InteractorStyle();

  void Initialize();

private:
  vtkSmartPointer<vtkBoxWidget2> BoxWidget;
};

#endif