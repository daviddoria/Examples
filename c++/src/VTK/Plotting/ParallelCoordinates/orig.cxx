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

  vtkSmartPointer<vtkTable> table =
    vtkSmartPointer<vtkTable>::New();
    
  vtkSmartPointer<vtkFloatArray> array1 =
    vtkSmartPointer<vtkFloatArray>::New();
  array1->SetName("Field 1");
  table->AddColumn(array1);
  
  vtkSmartPointer<vtkFloatArray> array2 =
    vtkSmartPointer<vtkFloatArray>::New();
  array2->SetName("Field 2");
  table->AddColumn(array2);
  
  vtkSmartPointer<vtkFloatArray> array3 =
    vtkSmartPointer<vtkFloatArray>::New();
  array3->SetName("Field 3");
  table->AddColumn(array3);
  
  vtkSmartPointer<vtkFloatArray> array4 =
    vtkSmartPointer<vtkFloatArray>::New();
  array4->SetName("Field 4");
  table->AddColumn(array4);
  
  // Generate 4D data points [i, cos(i), sin(i), tan(i)]
  int numPoints = 200;
  
  table->SetNumberOfRows(numPoints);
  for (int i = 0; i < numPoints; ++i)
    {
    table->SetValue(i, 0, i);
    table->SetValue(i, 1, cos(i));
    table->SetValue(i, 2, sin(i));
    table->SetValue(i, 3, tan(i));
    }

  chart->GetPlot(0)->SetInput(table);

  view->GetRenderWindow()->SetMultiSamples(0);

  view->GetInteractor()->Initialize();
  view->GetInteractor()->Start();

  return EXIT_SUCCESS;
}
