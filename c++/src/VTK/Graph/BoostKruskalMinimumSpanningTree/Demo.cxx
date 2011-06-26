#include <vtkSmartPointer.h>
#include <vtkGraph.h>
#include <vtkTree.h>
#include <vtkBoostKruskalMinimumSpanningTree.h>
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
//#include <vtkExtractSelection.h>
#include <vtkExtractSelectedGraph.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkGraphReader> reader = 
      vtkSmartPointer<vtkGraphReader>::New();
  reader->SetFileName(argv[1]);
  reader->Update();
  
  cout << "Number of Weights: " << vtkDoubleArray::SafeDownCast(reader->GetOutput()->GetEdgeData()->GetArray("Weights"))->GetNumberOfTuples() << endl;
  
  for(vtkIdType i = 0; i < reader->GetOutput()->GetPoints()->GetNumberOfPoints(); i++)
    {
    double p[3];
    reader->GetOutput()->GetPoints()->GetPoint(i, p);
    cout << "point " << i << " : " << p[0] << " " << p[1] << " " << p[2] << endl;
    }
    
  //setup the minimum spanning tree filter
  vtkSmartPointer<vtkBoostKruskalMinimumSpanningTree> kruskalFilter = 
      vtkSmartPointer<vtkBoostKruskalMinimumSpanningTree>::New();
  kruskalFilter->SetInputConnection(reader->GetOutputPort());
  kruskalFilter->SetEdgeWeightArrayName("Weights");
  kruskalFilter->Update();

  cout << "Reader output is a " << reader->GetOutput()->GetClassName() << endl;
  //minimumSpanningTreeFilter->GetOutput()->SetPoints(reader->GetOutput()->GetPoints());
  //vtkSelection* selection = minimumSpanningTreeFilter->GetOutput();
  
  /*
  vtkSmartPointer<vtkExtractSelection> extractSelection = 
      vtkSmartPointer<vtkExtractSelection>::New();
  extractSelection->SetInput(0, reader->GetOutput());
  extractSelection->SetInput(1, kruskalFilter->GetOutput());
  extractSelection->Update();
  */
  
  vtkSmartPointer<vtkExtractSelectedGraph> extractSelectedGraph = 
      vtkSmartPointer<vtkExtractSelectedGraph>::New();
  extractSelectedGraph->SetInput(0, reader->GetOutput());
  extractSelectedGraph->SetInput(1, kruskalFilter->GetOutput());
  extractSelectedGraph->Update();
  
  
  cout << "selection output is type: " << extractSelectedGraph->GetOutput()->GetClassName() << endl;
  
  cout << "selection output has " << extractSelectedGraph->GetOutput()->GetNumberOfVertices() << " vertices." << endl;
  cout << "selection output has " << extractSelectedGraph->GetOutput()->GetNumberOfEdges() << " edges." << endl;
  
  vtkSmartPointer<vtkTree> mst =
      vtkSmartPointer<vtkTree>::New();
  mst->CheckedShallowCopy(extractSelectedGraph->GetOutput());
  
  cout << "mst is of type " << mst->GetClassName() << endl;
  cout << "mst has " << mst->GetNumberOfVertices() << " vertices." << endl;
  cout << "mst has " << mst->GetNumberOfEdges() << " edges." << endl;
  
  //output information about the minimum spanning tree
  //vtkDataObject* dataObject = extractSelection->GetOutput();
  //vtkGraph* mst = vtkGraph::SafeDownCast(dataObject);
  //vtkTree* mst = vtkTree::SafeDownCast(extractSelectedGraph->GetOutput());
  //vtkUndirectedGraph* mst = vtkUndirectedGraph::SafeDownCast(extractSelection->GetOutput());
  //vtkDirectedGraph* mst = vtkDirectedGraph::SafeDownCast(extractSelection->GetOutput());
      
  //cout << "Number of vertices: " << vtkGraph::SafeDownCast(extractSelection->GetOutput())->GetNumberOfVertices() << endl;
  //cout << "Number of edges: " << vtkGraph::SafeDownCast(extractSelection->GetOutput())->GetNumberOfEdges() << endl;
  //cout << "Number of vertices: " << vtkTree::SafeDownCast(extractSelection->GetOutput())->GetNumberOfVertices() << endl;
  //cout << "Number of edges: " << vtkTree::SafeDownCast(extractSelection->GetOutput())->GetNumberOfEdges() << endl;
      
  vtkSmartPointer<vtkGraphToPolyData> graphToPolyData = 
      vtkSmartPointer<vtkGraphToPolyData>::New();
  graphToPolyData->SetInputConnection(extractSelectedGraph->GetOutputPort());
  graphToPolyData->Update();
  
  cout << "Number of points: " << graphToPolyData->GetOutput()->GetNumberOfPoints() << endl;
  cout << "Number of cells: " << graphToPolyData->GetOutput()->GetNumberOfCells() << endl;
  
    //setup actor and mapper
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(graphToPolyData->GetOutputPort());
 
  for(vtkIdType i = 0; i < graphToPolyData->GetOutput()->GetNumberOfPoints(); i++)
    {
    double p[3];
    graphToPolyData->GetOutput()->GetPoint(i, p);
    cout << "point " << i << " : " << p[0] << " " << p[1] << " " << p[2] << endl;
    }
    
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
