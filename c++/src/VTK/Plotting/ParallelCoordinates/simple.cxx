#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkChartParallelCoordinates.h>
#include <vtkPlot.h>
#include <vtkTable.h>
#include <vtkFloatArray.h>
#include <vtkContextView.h>
#include <vtkContextScene.h>
#include <vtkRenderWindowInteractor.h>

int main(int, char*[])
{
  // Set up a 2D scene, add an XY chart to it
  vtkSmartPointer<vtkContextView> view =
    vtkSmartPointer<vtkContextView>::New();
  view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
  view->GetRenderWindow()->SetSize(800, 600);
  vtkSmartPointer<vtkChartParallelCoordinates> chart =
    vtkSmartPointer<vtkChartParallelCoordinates>::New();
  view->GetScene()->AddItem(chart);

  // Create a table with some points in it...
  vtkSmartPointer<vtkTable> table =
    vtkSmartPointer<vtkTable>::New();
  vtkSmartPointer<vtkFloatArray> arrX =
    vtkSmartPointer<vtkFloatArray>::New();
  arrX->SetName("Field 1");
  table->AddColumn(arrX);
  vtkSmartPointer<vtkFloatArray> arrC =
    vtkSmartPointer<vtkFloatArray>::New();
  arrC->SetName("Field 2");
  table->AddColumn(arrC);
  vtkSmartPointer<vtkFloatArray> arrS =
    vtkSmartPointer<vtkFloatArray>::New();
  arrS->SetName("Field 3");
  table->AddColumn(arrS);
  vtkSmartPointer<vtkFloatArray> arrS2 =
    vtkSmartPointer<vtkFloatArray>::New();
  arrS2->SetName("Field 4");
  table->AddColumn(arrS2);
  // Test charting with a few more points...
  
  table->SetNumberOfRows(1);
  for (int i = 0; i < 1; ++i)
    {
    table->SetValue(i, 0, 0);
    table->SetValue(i, 1, 1);
    table->SetValue(i, 2, 2);
    table->SetValue(i, 3, 3);
    }

  chart->GetPlot(0)->SetInput(table);

  view->GetRenderWindow()->SetMultiSamples(0);

  view->GetInteractor()->Initialize();
  view->GetInteractor()->Start();

  return EXIT_SUCCESS;
}
