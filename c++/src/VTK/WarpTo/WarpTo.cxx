//Author: Chung Kai Lun Peter
#include <vtkSmartPointer.h>
#include <vtkLineSource.h>
#include <vtkTubeFilter.h>
#include <vtkDataSetMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkWarpTo.h>

int main(int, char*[])
{
  // Create the RenderWindow, Renderer and both Actors
  vtkSmartPointer<vtkRenderer> ren1 = 
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renWin = 
    vtkSmartPointer<vtkRenderWindow>::New();
  renWin->AddRenderer(ren1);
  vtkSmartPointer<vtkRenderWindowInteractor> iren = 
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  iren->SetRenderWindow(renWin);

  // Create a line
  vtkSmartPointer<vtkLineSource> lineSource = 
    vtkSmartPointer<vtkLineSource>::New();
  lineSource->SetPoint1(0.0, 0.0, 0.0);
  lineSource->SetPoint2(0.0, 1.0, 0.0);
 
  // Create a tube (cylinder) around the line
  vtkSmartPointer<vtkTubeFilter> tubeFilter = 
    vtkSmartPointer<vtkTubeFilter>::New();
  tubeFilter->SetInputConnection(lineSource->GetOutputPort());
  tubeFilter->SetRadius(.25); //default is .5
  tubeFilter->SetNumberOfSides(50);
  tubeFilter->Update();
  
  vtkSmartPointer<vtkWarpTo> warpTo = 
    vtkSmartPointer<vtkWarpTo>::New();
  warpTo->SetInputConnection(tubeFilter->GetOutputPort());
  //arrowWT->SetPosition(5, 0, 5);
  warpTo->SetPosition(5, .5, 0);
  warpTo->SetScaleFactor(0.85);
  warpTo->AbsoluteOn();

  vtkSmartPointer<vtkDataSetMapper> arrowMapper = 
    vtkSmartPointer<vtkDataSetMapper>::New();
  arrowMapper->SetInput(warpTo->GetOutput());
  arrowMapper->ScalarVisibilityOff();
  
  vtkSmartPointer<vtkActor> arrowActor = 
    vtkSmartPointer<vtkActor>::New();
  arrowActor->SetMapper(arrowMapper);
  
  ren1->AddActor(arrowActor);

  iren->Initialize();
  iren->Start();

  return EXIT_SUCCESS;
}