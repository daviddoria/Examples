#include <vtkSmartPointer.h>
#include <vtkDataSetAttributes.h>
#include <vtkUnsignedCharArray.h>
#include <vtkGraphLayoutView.h>
#include <vtkMutableDirectedGraph.h>
#include <vtkTree.h>
#include <vtkTreeDFSIterator.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderedGraphRepresentation.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkMutableDirectedGraph> graph =
    vtkSmartPointer<vtkMutableDirectedGraph>::New();


  // Create a tree
  vtkIdType v1 = graph->AddVertex();
  vtkIdType v2 = graph->AddChild(v1);
  graph->AddChild(v1);
  graph->AddChild(v2);

  vtkSmartPointer<vtkTree> tree =
    vtkSmartPointer<vtkTree>::New();
  tree->CheckedShallowCopy(graph);

  // Create the color array
  vtkSmartPointer<vtkUnsignedCharArray> edgeColors =
    vtkSmartPointer<vtkUnsignedCharArray>::New();
  edgeColors->SetNumberOfComponents(3);
  edgeColors->SetName("EdgeColors");

  unsigned char red[3] = {255,0,0};
  unsigned char green[3] = {0,255,0};
  unsigned char blue[3] = {0,0,255};
  unsigned char yellow[3] = {0,255,255};

  vtkSmartPointer<vtkUnsignedCharArray> vertexColors =
    vtkSmartPointer<vtkUnsignedCharArray>::New();
  vertexColors->SetNumberOfComponents(3);
  vertexColors->SetName("VertexColors");

  vertexColors->InsertNextTupleValue(red);
  vertexColors->InsertNextTupleValue(green);
  vertexColors->InsertNextTupleValue(blue);
  vertexColors->InsertNextTupleValue(yellow);

  edgeColors->InsertNextTupleValue(red);
  edgeColors->InsertNextTupleValue(green);
  edgeColors->InsertNextTupleValue(blue);

  // Add the color array to the graph
  tree->GetVertexData()->SetScalars(vertexColors);
  tree->GetEdgeData()->SetScalars(edgeColors);

  vtkSmartPointer<vtkGraphLayoutView> graphLayoutView =
    vtkSmartPointer<vtkGraphLayoutView>::New();
  graphLayoutView->AddRepresentationFromInput(tree);
  graphLayoutView->SetLayoutStrategyToTree();
  graphLayoutView->ColorVerticesOn();
  graphLayoutView->SetColorModeToDefault();
  //graphLayoutView->SetColorModeToMapScalars();
  graphLayoutView->ResetCamera();
  graphLayoutView->Render();
  graphLayoutView->GetInteractor()->Start();

  return EXIT_SUCCESS;
}
