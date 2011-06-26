#include <vtkTable.h>
#include <vtkStringArray.h>
#include <vtkTableToGraph.h>
#include <vtkVariant.h>
#include <vtkVariantArray.h>
#include <vtkSmartPointer.h>
#include <vtkGraphLayoutStrategy.h>
#include <vtkGraphLayoutView.h>
#include <vtkGraphWriter.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSimple2DLayoutStrategy.h>
#include <vtkAdjacencyMatrixToEdgeTable.h>
#include <vtkDenseArray.h>
#include <vtkArrayData.h>
#include <vtkArrayPrint.h>
#include <vtkDataSetAttributes.h>

#include <sstream>

int main(int, char *[])
{
   vtkSmartPointer<vtkDenseArray<double> > array =
    vtkSmartPointer<vtkDenseArray<double> >::New();

  array->Resize(3,3);

  /*
  // only single direction links

  // diagonal
  array->SetValue(0, 0, 0);
  array->SetValue(1, 1, 0);
  array->SetValue(2, 2, 0);

  // lower triangular
  array->SetValue(0, 1, 10);
  array->SetValue(0, 2, 15);
  array->SetValue(1, 2, 0);

  // lower triangular
  array->SetValue(1, 0, 0);
  array->SetValue(2, 0, 0);
  array->SetValue(2, 1, 0);
  */

  // symmetric

  // diagonal
  array->SetValue(0, 0, 0);
  array->SetValue(1, 1, 0);
  array->SetValue(2, 2, 0);

  // lower triangular
  array->SetValue(0, 1, 10);
  array->SetValue(0, 2, 15);
  array->SetValue(1, 2, 0);

  // lower triangular
  array->SetValue(1, 0, 10);
  array->SetValue(2, 0, 15);
  array->SetValue(2, 1, 0);


  vtkPrintMatrixFormat(std::cout, array.GetPointer());

  array->SetDimensionLabel(0, "rows");
  array->SetDimensionLabel(1, "columns");

  vtkSmartPointer<vtkStringArray> vertexLabels =
    vtkSmartPointer<vtkStringArray>::New();
  vertexLabels->SetNumberOfComponents(1);
  vertexLabels->SetName("VertexLabels");
  for(unsigned int i = 0; i < 3; i++)
  {
    std::stringstream ss;
    ss << i;
    vertexLabels->InsertNextValue(ss.str().c_str());
  }
  vtkSmartPointer<vtkArrayData> arrayData =
    vtkSmartPointer<vtkArrayData>::New();
  arrayData->AddArray(array);

  vtkSmartPointer<vtkAdjacencyMatrixToEdgeTable> adjacencyMatrixToEdgeTable =
    vtkSmartPointer<vtkAdjacencyMatrixToEdgeTable>::New();
  adjacencyMatrixToEdgeTable->SetInputConnection(arrayData->GetProducerPort());
  adjacencyMatrixToEdgeTable->Update();

  adjacencyMatrixToEdgeTable->GetOutput()->Dump();


  vtkSmartPointer<vtkTableToGraph> tableToGraph =
    vtkSmartPointer<vtkTableToGraph>::New();
  tableToGraph->SetInputConnection(adjacencyMatrixToEdgeTable->GetOutputPort());
  tableToGraph->AddLinkVertex("rows");
  tableToGraph->AddLinkVertex("columns");

  tableToGraph->AddLinkEdge("rows", "columns");
  tableToGraph->Update();

  vtkSmartPointer<vtkMutableUndirectedGraph> g =
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  g->ShallowCopy(tableToGraph->GetOutput());
  g->GetVertexData()->AddArray(vertexLabels);

  vtkSmartPointer<vtkGraphLayoutView> graphLayoutView =
    vtkSmartPointer<vtkGraphLayoutView>::New();
  graphLayoutView->AddRepresentationFromInput(g);
  graphLayoutView->SetVertexLabelVisibility(true);
  graphLayoutView->SetEdgeLabelVisibility(true);
  graphLayoutView->SetEdgeLabelArrayName("values");
  graphLayoutView->SetVertexLabelArrayName("VertexLabels");
  graphLayoutView->ResetCamera();
  graphLayoutView->Render();

  graphLayoutView->GetInteractor()->Start();

  return EXIT_SUCCESS;
}