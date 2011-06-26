#include <vtkSmartPointer.h>
#include <vtkAxis.h>
#include <vtkTimerLog.h>
#include <vtkMath.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkIdList.h>
#include <vtkKdTreePointLocator.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkChartXY.h>
#include <vtkPlot.h>
#include <vtkTable.h>
#include <vtkFloatArray.h>
#include <vtkContextView.h>
#include <vtkContextScene.h>
#include <vtkSphereSource.h>
#include <vtkPointSource.h>
 
#include <vector>
 
void RandomPointInBounds(vtkPolyData* polydata, double p[3]);
double TimeKDTree(vtkPolyData* polydata, int maxPoints, int numberOfTrials);
 
int main(int argc, char *argv[])
{
  /*
  vtkSmartPointer<vtkXMLPolyDataReader> reader =
    vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(argv[1]);
  reader->Update();
  */
  /*
  vtkSmartPointer<vtkSphereSource> reader =
    vtkSmartPointer<vtkSphereSource>::New();
  reader->SetThetaResolution(30);
  reader->SetPhiResolution(30);
  reader->Update();
  */
  vtkSmartPointer<vtkPointSource> reader =
    vtkSmartPointer<vtkPointSource>::New();
  reader->SetNumberOfPoints(10000);
  reader->SetRadius(5);
  reader->Update();
  
  std::cout << "Timing octree..." << std::endl;
  std::vector<std::pair<int, double> > results;
  int numberOfTrials = 10000;
  for(int i = 1; i < 20; i++)
    {
    double t = TimeKDTree(reader->GetOutput(), i, numberOfTrials);
    std::pair<int, double> result(i,t);
    results.push_back(result);
    }
 
  // Create a table with some points in it
  vtkSmartPointer<vtkTable> table =
    vtkSmartPointer<vtkTable>::New();
 
  vtkSmartPointer<vtkFloatArray> maxPointsPerRegion =
    vtkSmartPointer<vtkFloatArray>::New();
  maxPointsPerRegion->SetName("MaxPointsPerRegion");
  table->AddColumn(maxPointsPerRegion);
 
  vtkSmartPointer<vtkFloatArray> runtime =
    vtkSmartPointer<vtkFloatArray>::New();
  runtime->SetName("Run time");
  table->AddColumn(runtime);
 
  // Fill in the table with some example values
  int numPoints = results.size();
  table->SetNumberOfRows(numPoints);
  for(int i = 0; i < numPoints; ++i)
    {
    table->SetValue(i, 0, results[i].first);
    table->SetValue(i, 1, results[i].second);
    std::cout << "Put " << results[i].first << " " << results[i].second << " in the table." << std::endl;
    }
 
  // Set up the view
  vtkSmartPointer<vtkContextView> view =
    vtkSmartPointer<vtkContextView>::New();
  view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
 
  // Add multiple line plots, setting the colors etc
  vtkSmartPointer<vtkChartXY> chart =
    vtkSmartPointer<vtkChartXY>::New();
  view->GetScene()->AddItem(chart);
  vtkPlot *line = chart->AddPlot(vtkChart::LINE);
  line->SetInput(table, 0, 1);
  line->SetColor(0, 255, 0, 255);
  line->SetWidth(3.0);
  line->GetXAxis()->SetTitle("Max Points Per Region");
  line->GetYAxis()->SetTitle("Run time");
  //line->GetYAxis()->AutoScale();
  //line->GetYAxis()->SetRange(0,0.02);
 
  // Set up an interactor and start
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(view->GetRenderWindow());
  renderWindowInteractor->Initialize();
  renderWindowInteractor->Start();
 
 
  return EXIT_SUCCESS;
}
 
void RandomPointInBounds(vtkPolyData* polydata, double p[3])
{
  double bounds[6];
  polydata->GetBounds(bounds);
 
  double x = bounds[0] + (bounds[1] - bounds[0]) * vtkMath::Random(0.0,1.0);
  double y = bounds[2] + (bounds[3] - bounds[2]) * vtkMath::Random(0.0,1.0);
  double z = bounds[4] + (bounds[5] - bounds[4]) * vtkMath::Random(0.0,1.0);
 
  p[0] = x;
  p[1] = y;
  p[2] = z;
}
 
double TimeKDTree(vtkPolyData* polydata, int maxLevel, int numberOfTrials)
{
  vtkSmartPointer<vtkTimerLog> timer =
    vtkSmartPointer<vtkTimerLog>::New();
  timer->StartTimer();
 
  vtkMath::RandomSeed(time(NULL));
 
  // Create the tree
  vtkSmartPointer<vtkKdTreePointLocator> kdtree =
    vtkSmartPointer<vtkKdTreePointLocator>::New();
  kdtree->SetDataSet(polydata);
  kdtree->AutomaticOff();
  kdtree->SetMaxLevel(maxLevel);
  kdtree->BuildLocator();
 
//  std::cout << "With maxLevel = " << maxLevel << " there are " << kdtree->GetNumberOfLeafNodes() << " leaf nodes." << std::endl;
 
  for(int i = 0; i < numberOfTrials; i++)
    {
    double p[3];
    RandomPointInBounds(polydata, p);
    vtkIdType iD = kdtree->FindClosestPoint(p);
    }
 
  timer->StopTimer();
 
  std::cout << "KDTree took " << timer->GetElapsedTime() << std::endl;
 
  return timer->GetElapsedTime();
}