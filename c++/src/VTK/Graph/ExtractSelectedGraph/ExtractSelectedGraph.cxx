#include <vtkSmartPointer.h>
#include <vtkDataSetAttributes.h>
#include <vtkDoubleArray.h>
#include <vtkExtractSelectedGraph.h>
#include <vtkIdTypeArray.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkSelection.h>
#include <vtkSelectionNode.h>
#include <vtkDataSetAttributes.h>
#include <vtkDoubleArray.h>
#include <vtkGraphLayoutView.h>
#include <vtkIntArray.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkStringArray.h>
 
int main(int, char *[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g = 
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();
 
  //create 3 vertices
  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();
  vtkIdType v3 = g->AddVertex();
 
  //add edges
  g->AddEdge(v1, v2);
  g->AddEdge(v2, v3);
  g->AddEdge(v1, v3);
 
  //create the vertex label array
  vtkSmartPointer<vtkStringArray> vertexLabels =
    vtkSmartPointer<vtkStringArray>::New();
  vertexLabels->SetNumberOfComponents(1);
  vertexLabels->SetName("VertexLabels");
  vertexLabels->InsertNextValue("Label0");
  vertexLabels->InsertNextValue("Label1");
  vertexLabels->InsertNextValue("Label2");
  g->GetVertexData()->AddArray(vertexLabels);
 
  std::cout << "Number of vertices: "
            << g->GetNumberOfVertices() << std::endl;
  std::cout << "Number of edges: "
            << g->GetNumberOfEdges() << std::endl;
 
  vtkSmartPointer<vtkIdTypeArray> ids = 
    vtkSmartPointer<vtkIdTypeArray>::New();
  ids->InsertNextValue(0);
  ids->InsertNextValue(1);
 
  vtkSmartPointer<vtkSelection> selection =
    vtkSmartPointer<vtkSelection>::New();
 
  vtkSmartPointer<vtkSelectionNode> node =
    vtkSmartPointer<vtkSelectionNode>::New();
  selection->AddNode(node);
  node->SetSelectionList(ids);
  node->SetContentType(vtkSelectionNode::INDICES);
  node->SetFieldType(vtkSelectionNode::VERTEX);
 
  vtkSmartPointer<vtkExtractSelectedGraph> extractSelectedGraph = 
    vtkSmartPointer<vtkExtractSelectedGraph>::New();
  extractSelectedGraph->SetInput(0, g);
  extractSelectedGraph->SetInput(1, selection);
  extractSelectedGraph->Update();
 
  std::cout << "Number of vertices: "
            << extractSelectedGraph->GetOutput()->GetNumberOfVertices()
            << std::endl;
  std::cout << "Number of edges: "
            << extractSelectedGraph->GetOutput()->GetNumberOfEdges()
            << std::endl;
 
  vtkSmartPointer<vtkGraphLayoutView> graphLayoutView = 
    vtkSmartPointer<vtkGraphLayoutView>::New();
  graphLayoutView->AddRepresentationFromInput(g);
 
  graphLayoutView->SetVertexLabelVisibility(true);
  graphLayoutView->SetEdgeLabelVisibility(true);
  graphLayoutView->SetVertexLabelArrayName("VertexLabels");
  graphLayoutView->ResetCamera();
  graphLayoutView->Render();
  graphLayoutView->GetInteractor()->Start();
 
  graphLayoutView->AddRepresentationFromInput(extractSelectedGraph->GetOutput());
  graphLayoutView->ResetCamera();
  graphLayoutView->Render();
  graphLayoutView->GetInteractor()->Start();
 
  return EXIT_SUCCESS;
}
