#include <vtkSmartPointer.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkBoostPrimMinimumSpanningTree.h>
#include <vtkTree.h>
#include <vtkDoubleArray.h>
#include <vtkDataSetAttributes.h>
#include <vtkPoints.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g = 
      vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  
  //create 3 vertices
  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();
  vtkIdType v3 = g->AddVertex();
  
  //create a fully connected graph
  g->AddEdge(v1, v2);
  g->AddEdge(v2, v3);
  g->AddEdge(v1, v3);
  
  //create the edge weight array
  vtkSmartPointer<vtkDoubleArray> weights = 
      vtkSmartPointer<vtkDoubleArray>::New();
  weights->SetNumberOfComponents(1);
  weights->SetName("Weights");
  
  //set the edge weights
  weights->InsertNextValue(1.0);
  weights->InsertNextValue(1.0);
  weights->InsertNextValue(2.0);
  
  //add the edge weight array to the graph
  g->GetEdgeData()->AddArray(weights);
  
  //create 3 points - one for each vertex
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(1.0, 0.0, 0.0);
  points->InsertNextPoint(0.0, 1.0, 0.0);
  points->InsertNextPoint(0.0, 0.0, 1.0);
   
  //add the coordinates of the points to the graph
  g->SetPoints(points);
  
  //output original graph info
  cout << "Number of vertices: " << g->GetNumberOfVertices() << endl;
  cout << "Number of edges: " << g->GetNumberOfEdges() << endl;
  cout << "Number of points: " << g->GetPoints()->GetNumberOfPoints() << endl;
  
  {
  //retrieve points
  double p[3];
  g->GetPoints()->GetPoint(0,p);
  cout << "p: " << p[0] << " " << p[1] << " " << p[2] << endl;
  }
 
  
  //setup the minimum spanning tree filter
  vtkSmartPointer<vtkBoostPrimMinimumSpanningTree> minimumSpanningTreeFilter = 
      vtkSmartPointer<vtkBoostPrimMinimumSpanningTree>::New();
  minimumSpanningTreeFilter->SetOriginVertex(v1);
  minimumSpanningTreeFilter->SetInput(g);
  minimumSpanningTreeFilter->SetEdgeWeightArrayName("Weights");
  
  //compute the minimum spanning tree
  minimumSpanningTreeFilter->Update();
  
  //get the output tree
  vtkSmartPointer<vtkTree> minimumSpanningTree = 
      vtkSmartPointer<vtkTree>::New();
  minimumSpanningTree->ShallowCopy(minimumSpanningTreeFilter->GetOutput());
  
  //output information about the minimum spanning tree
  cout << "Number of vertices: " << minimumSpanningTree->GetNumberOfVertices() << endl;
  cout << "Number of edges: " << minimumSpanningTree->GetNumberOfEdges() << endl;
  cout << "Number of points: " << minimumSpanningTree->GetPoints()->GetNumberOfPoints() << endl;
  
  {
  //retrieve points
  double p[3];
  minimumSpanningTree->GetPoints()->GetPoint(0,p);
  cout << "p: " << p[0] << " " << p[1] << " " << p[2] << endl;
  }
  
  return EXIT_SUCCESS;
}
