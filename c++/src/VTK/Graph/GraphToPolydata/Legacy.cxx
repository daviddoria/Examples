#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkGraphToPolyData.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkGraphLayout.h>
#include <vtkAssignCoordinatesLayoutStrategy.h>

int main ( int argc, char *argv[] )
{
  vtkSmartPointer<vtkMutableUndirectedGraph> G = vtkSmartPointer<vtkMutableUndirectedGraph>::New();

  vtkIdType V1 = G->AddVertex();
  vtkIdType V2 = G->AddVertex();
  vtkIdType V3 = G->AddVertex();
  vtkIdType V4 = G->AddVertex();

  G->AddEdge ( V1, V2 );
  G->AddEdge ( V1, V3 );
  G->AddEdge ( V1, V4 );

  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
  Points->InsertNextPoint(0.0, 0.0, 0.0);
  Points->InsertNextPoint(1.0, 0.0, 0.0);
  Points->InsertNextPoint(0.0, 1.0, 0.0);
  Points->InsertNextPoint(0.0, 0.0, 1.0);
  
  G->SetPoints(Points);
  
  //vtkSmartPointer<vtkAssignCoordinatesLayoutStrategy> AssignCoordinates = vtkSmartPointer<vtkAssignCoordinatesLayoutStrategy>::New();
  
  vtkSmartPointer<vtkGraphLayout> GraphLayout = vtkSmartPointer<vtkGraphLayout>::New();
  GraphLayout->SetInput(G);
  //GraphLayout->SetLayoutStrategy(AssignCoordinates);
  GraphLayout->Update();
      
  /*
  vtkSmartPointer<vtkGraphToPolyData> GraphToPolyData = vtkSmartPointer<vtkGraphToPolyData>::New();
  GraphToPolyData->SetInput(GraphLayout->GetOutput());
  GraphToPolyData->Update();
  
  vtkSmartPointer<vtkXMLPolyDataWriter> Writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  Writer->SetInput(GraphToPolyData->GetOutput());
  Writer->SetFileName("GraphPolyData.vtp");
  Writer->Write();
  */
  return 0;
}
