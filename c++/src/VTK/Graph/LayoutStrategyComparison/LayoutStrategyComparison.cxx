#include <vtkSmartPointer.h>
#include <vtkGraphToPolyData.h>
#include <vtkDataSetAttributes.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkGraphLayoutView.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkInteractorStyleRubberBand2D.h>
#include <vtkRemoveIsolatedVertices.h>

#include <vtkGraphLayoutStrategy.h>
#include <vtkRandomLayoutStrategy.h>
#include <vtkForceDirectedLayoutStrategy.h>
#include <vtkSimple2DLayoutStrategy.h>
#include <vtkFast2DLayoutStrategy.h>
#include <vtkClustering2DLayoutStrategy.h>
#include <vtkCircularLayoutStrategy.h>

#include <vector>

int main(int, char *[])
{
  // Create the first graph
  vtkSmartPointer<vtkMutableUndirectedGraph> g =
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();

  std::vector<vtkIdType> vertices;
  for(unsigned int i = 0; i < 100; i++)
    {
    vertices.push_back(g->AddVertex());
    }

  for(unsigned int i = 0 ; i < 100; i++)
    {
    g->AddEdge(rand() % vertices.size(), rand() % vertices.size());
    }

  vtkSmartPointer<vtkRemoveIsolatedVertices> filter =
    vtkSmartPointer<vtkRemoveIsolatedVertices>::New();
  filter->SetInputConnection(g->GetProducerPort());
  filter->Update();

  std::vector<vtkSmartPointer<vtkGraphLayoutStrategy> > strategies;

  int randomSeed = 100;
  // Randomly places vertices in a box
  vtkSmartPointer<vtkRandomLayoutStrategy> random =
    vtkSmartPointer<vtkRandomLayoutStrategy>::New();
  random->SetRandomSeed(randomSeed);
  strategies.push_back(random);

  // A layout in 3D or 2D simulating forces on edges
  vtkSmartPointer<vtkForceDirectedLayoutStrategy> forceDirected =
    vtkSmartPointer<vtkForceDirectedLayoutStrategy>::New();
  forceDirected->SetRandomSeed(randomSeed);
  strategies.push_back(forceDirected);

  // (the default) A simple 2D force directed layout
  vtkSmartPointer<vtkSimple2DLayoutStrategy> simple2D =
    vtkSmartPointer<vtkSimple2DLayoutStrategy>::New();
  simple2D->SetRandomSeed(randomSeed);
  strategies.push_back(simple2D);

  //A 2D force directed layout that's just like simple 2D but uses some techniques to cluster better. -
  vtkSmartPointer<vtkClustering2DLayoutStrategy> clustering2D =
    vtkSmartPointer<vtkClustering2DLayoutStrategy>::New();
  clustering2D->SetRandomSeed(randomSeed);
  strategies.push_back(clustering2D);

  // A linear-time 2D layout.
  vtkSmartPointer<vtkFast2DLayoutStrategy> fast2D =
    vtkSmartPointer<vtkFast2DLayoutStrategy>::New();
  fast2D->SetRandomSeed(randomSeed);
  strategies.push_back(fast2D);

  // Places vertices uniformly on a circle.
  vtkSmartPointer<vtkCircularLayoutStrategy> circular =
    vtkSmartPointer<vtkCircularLayoutStrategy>::New();
  // circular->SetRandomSeed(randomSeed); // Not applicable
  strategies.push_back(circular);

  // There will be one render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(300*strategies.size(), 300);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();

  vtkSmartPointer<vtkInteractorStyleRubberBand2D> style =
    vtkSmartPointer<vtkInteractorStyleRubberBand2D>::New();

  std::vector<vtkSmartPointer<vtkGraphLayoutView> > views;

  for(unsigned int i = 0; i < strategies.size(); i++)
    {
    double width = 1.0 / strategies.size();
    double viewport[4] = {static_cast<double>(i) * width, 0.0, static_cast<double>(i+1)*width, 1.0};

    vtkSmartPointer<vtkGraphLayoutView> graphLayoutView =
      vtkSmartPointer<vtkGraphLayoutView>::New();
    graphLayoutView->SetInteractor(renderWindowInteractor);
    graphLayoutView->SetInteractorStyle(style);
    graphLayoutView->SetLayoutStrategy(strategies[i]);
    //graphLayoutView->SetLayoutStrategy(strategies[i].c_str());
    //graphLayoutView->GetLayoutStrategy()->SetRandomSeed(100);
    graphLayoutView->SetRenderWindow(renderWindow);
    graphLayoutView->GetRenderer()->SetViewport(viewport);
    graphLayoutView->GetRenderer()->SetBackground(.3,.4,.5);
    graphLayoutView->AddRepresentationFromInput(filter->GetOutput());
    views.push_back(graphLayoutView);
    }

  renderWindowInteractor->Initialize();

  for(unsigned int i = 0; i < views.size(); i++)
    {
    views[i]->ResetCamera();
    views[i]->Render();
    }


  renderWindowInteractor->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
