#include <vtkSmartPointer.h>
#include <vtkBoostPrimMinimumSpanningTree.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkGraphToPolyData.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkGraphReader.h>
#include <vtkDataSetAttributes.h>
#include <vtkDoubleArray.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkGraphReader> reader = 
      vtkSmartPointer<vtkGraphReader>::New();
  reader->SetFileName(argv[1]);
  reader->Update();
  
  cout << "Number of Weights: " << vtkDoubleArray::SafeDownCast(reader->GetOutput()->GetEdgeData()->GetArray("Weights"))->GetNumberOfTuples() << endl;
  
  /*
  for(vtkIdType i = 0; i < reader->GetOutput()->GetPoints()->GetNumberOfPoints(); i++)
    {
    double p[3];
    reader->GetOutput()->GetPoints()->GetPoint(i, p);
    cout << "point " << i << " : " << p[0] << " " << p[1] << " " << p[2] << endl;
    }
  */
    
  //setup the minimum spanning tree filter
  vtkSmartPointer<vtkBoostPrimMinimumSpanningTree> minimumSpanningTreeFilter = 
      vtkSmartPointer<vtkBoostPrimMinimumSpanningTree>::New();
  minimumSpanningTreeFilter->SetOriginVertex(0);
  minimumSpanningTreeFilter->SetInputConnection(reader->GetOutputPort());
  minimumSpanningTreeFilter->SetEdgeWeightArrayName("Weights");
  minimumSpanningTreeFilter->Update();

  minimumSpanningTreeFilter->GetOutput()->SetPoints(reader->GetOutput()->GetPoints());
  
  //output information about the minimum spanning tree
  cout << "Number of vertices: " << minimumSpanningTreeFilter->GetOutput()->GetNumberOfVertices() << endl;
  cout << "Number of edges: " << minimumSpanningTreeFilter->GetOutput()->GetNumberOfEdges() << endl;
      
  vtkSmartPointer<vtkTree> tree = minimumSpanningTreeFilter->GetOutput();
  cout << "Number of vertices: " << tree->GetNumberOfVertices() << endl;
  cout << "Number of edges: " << tree->GetNumberOfEdges() << endl;
  
  
  vtkSmartPointer<vtkGraphToPolyData> graphToPolyData = 
      vtkSmartPointer<vtkGraphToPolyData>::New();
  graphToPolyData->SetInputConnection(minimumSpanningTreeFilter->GetOutputPort());
  graphToPolyData->Update();
  
  cout << "Number of points: " << graphToPolyData->GetOutput()->GetNumberOfPoints() << endl;
  cout << "Number of cells: " << graphToPolyData->GetOutput()->GetNumberOfCells() << endl;
  
    //setup actor and mapper
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(graphToPolyData->GetOutputPort());
 
  /*
  for(vtkIdType i = 0; i < graphToPolyData->GetOutput()->GetNumberOfPoints(); i++)
    {
    double p[3];
    graphToPolyData->GetOutput()->GetPoint(i, p);
    cout << "point " << i << " : " << p[0] << " " << p[1] << " " << p[2] << endl;
    }
  */
  
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
 
  //setup render window, renderer, and interactor
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
  renderer->AddActor(actor);
  
  renderWindow->Render();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
