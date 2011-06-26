#include <vtkSmartPointer.h>
#include <vtkRenderedGraphRepresentation.h>
#include <vtkIntArray.h>
#include <vtkDataSetAttributes.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkGraphWriter.h>
#include <vtkGraphLayoutView.h>
#include <vtkRenderWindowInteractor.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g =
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();

  //create the vertex ID array
  vtkSmartPointer<vtkIntArray> vertexIDs =
    vtkSmartPointer<vtkIntArray>::New();
  vertexIDs->SetNumberOfComponents(1);
  vertexIDs->SetName("label");

  std::vector<vtkIdType> vertices;
  for(unsigned int i = 0; i < 200; i++)
    {
    vertices.push_back(g->AddVertex());
    vertexIDs->InsertNextValue(i);
    }

  for(unsigned int i = 0; i < 100; i++)
    {
    g->AddEdge(rand() % vertices.size(), rand() % vertices.size());
    }

  //add the edge weight array to the graph
  g->GetVertexData()->AddArray(vertexIDs);

  vtkSmartPointer<vtkGraphLayoutView> graphLayoutView =
    vtkSmartPointer<vtkGraphLayoutView>::New();
  std::cout << "Vertex label name: " << graphLayoutView->GetVertexLabelArrayName () << std::endl;

  graphLayoutView->AddRepresentationFromInput(g);
  std::cout << "Vertex label name: " << graphLayoutView->GetVertexLabelArrayName () << std::endl;

  graphLayoutView->SetVertexLabelArrayName("label");
  vtkRenderedGraphRepresentation::SafeDownCast(graphLayoutView->GetRepresentation())->HideVertexLabelsOnInteractionOn();
  graphLayoutView->SetVertexLabelVisibility(true);
  graphLayoutView->ResetCamera();
  graphLayoutView->Render();
  graphLayoutView->GetInteractor()->Start();

  return EXIT_SUCCESS;
}
