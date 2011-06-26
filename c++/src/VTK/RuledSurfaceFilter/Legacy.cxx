#include <vtkSmartPointer.h>
#include <vtkRuledSurfaceFilter.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCellArray.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkAppendPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkRuledSurfaceFilter> ruledSurfaceFilter =
      vtkSmartPointer<vtkRuledSurfaceFilter>::New();

  vtkSmartPointer<vtkRenderer> ren1 = 
    vtkSmartPointer<vtkRenderer>::New();
    
  vtkSmartPointer<vtkRenderWindow> renWin = 
    vtkSmartPointer<vtkRenderWindow>::New();
  renWin->AddRenderer(ren1);
  
  vtkSmartPointer<vtkRenderWindowInteractor> iren = 
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  iren->SetRenderWindow(renWin);

  
  // Create first line.

  vtkSmartPointer<vtkPoints> points = 
    vtkSmartPointer<vtkPoints>::New();
  points->InsertPoint(0, 0, 0, 0);
  points->InsertPoint(1, 1, 0, 0);
  points->InsertPoint(2, 1, 1, 0);
  points->InsertPoint(3, 2, 1, 0);

  vtkSmartPointer<vtkCellArray> lines =
    vtkSmartPointer<vtkCellArray>::New();
  lines->InsertNextCell(4);//number of points
  lines->InsertCellPoint(0);
  lines->InsertCellPoint(1);
  lines->InsertCellPoint(2);
  lines->InsertCellPoint(3);

  vtkSmartPointer<vtkPolyData> profile =
    vtkSmartPointer<vtkPolyData>::New();
  profile->SetPoints(points);
  profile->SetLines(lines);

  vtkSmartPointer<vtkTransform> xfm = 
    vtkSmartPointer<vtkTransform>::New();
  xfm->Translate(0, 0, 8);
  xfm->RotateZ(90);

  vtkSmartPointer<vtkTransformPolyDataFilter> xfmPd =
    vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  xfmPd->SetInput(profile);
  xfmPd->SetTransform(xfm);

  vtkSmartPointer<vtkAppendPolyData> appendPD=
    vtkSmartPointer<vtkAppendPolyData>::New();
  appendPD->AddInput(profile);
  appendPD->AddInput(xfmPd->GetOutput());

  // extrude profile to make wall

  vtkSmartPointer<vtkRuledSurfaceFilter> extrude =
    vtkSmartPointer<vtkRuledSurfaceFilter>::New();
  extrude->SetInputConnection(appendPD->GetOutputPort());
  extrude->SetResolution(51, 51);
  extrude->SetRuledModeToResample();

  vtkSmartPointer<vtkPolyDataMapper> map =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  map->SetInputConnection(extrude->GetOutputPort());

  vtkSmartPointer<vtkActor> wall=
    vtkSmartPointer<vtkActor>::New();
  wall->SetMapper(map);
  wall->GetProperty()->SetColor(0.3800, 0.7000, 0.1600);


  // Add the actors to the renderer, set the background and size

  ren1->AddActor(wall);
  ren1->SetBackground(1, 1, 1);

  renWin->Render();
  iren->Start();

  return EXIT_SUCCESS;
}
