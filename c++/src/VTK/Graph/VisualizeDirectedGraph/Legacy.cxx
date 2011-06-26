#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkGraphToPolyData.h>
#include <vtkGlyph3D.h>
#include <vtkGraphLayout.h>
#include <vtkSimple2DLayoutStrategy.h>
#include <vtkGlyphSource2D.h>
#include <vtkMutableDirectedGraph.h>
#include <vtkGraphWriter.h>
#include <vtkGraphLayoutView.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkActor.h>
#include <vtkRenderer.h>

int main ( int argc, char *argv[] )
{
  vtkSmartPointer<vtkMutableDirectedGraph> g = 
      vtkSmartPointer<vtkMutableDirectedGraph>::New();

  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();

  g->AddEdge ( v1, v2 );
  g->AddEdge ( v2, v1 );
  
  // Do layout manually before handing graph to the view.
  // This allows us to know the positions of edge arrows.
  vtkSmartPointer<vtkGraphLayoutView> graphLayoutView = 
      vtkSmartPointer<vtkGraphLayoutView>::New();
    
  vtkSmartPointer<vtkGraphLayout> layout = 
      vtkSmartPointer<vtkGraphLayout>::New();
  vtkSmartPointer<vtkSimple2DLayoutStrategy> strategy = 
      vtkSmartPointer<vtkSimple2DLayoutStrategy>::New();
  layout->SetInput(g);
  layout->SetLayoutStrategy(strategy);
  graphLayoutView->SetLayoutStrategyToPassThrough();

  // Add the graph to the view. This will render vertices and edges,
  // but not edge arrows.
  graphLayoutView->AddRepresentationFromInputConnection(layout->GetOutputPort());

  // Manually create an actor containing the glyphed arrows.
  vtkSmartPointer<vtkGraphToPolyData> graphToPoly = 
      vtkSmartPointer<vtkGraphToPolyData>::New();
  graphToPoly->SetInputConnection(layout->GetOutputPort());
  graphToPoly->EdgeGlyphOutputOn();

  // Set the position (0: edge start, 1: edge end) where
  // the edge arrows should go.
  graphToPoly->SetEdgeGlyphPosition(0.8);

  // Make a simple edge arrow for glyphing.
  vtkSmartPointer<vtkGlyphSource2D> arrowSource = 
      vtkSmartPointer<vtkGlyphSource2D>::New();
  arrowSource->SetGlyphTypeToEdgeArrow();
  arrowSource->SetScale(0.3);
  arrowSource->Update();

  // Use Glyph3D to repeat the glyph on all edges.
  vtkSmartPointer<vtkGlyph3D> arrowGlyph = 
      vtkSmartPointer<vtkGlyph3D>::New();
  arrowGlyph->SetInputConnection(0, graphToPoly->GetOutputPort(1));
  arrowGlyph->SetInputConnection(1, arrowSource->GetOutputPort());
  
  // Add the edge arrow actor to the view.
  vtkSmartPointer<vtkPolyDataMapper> arrowMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  arrowMapper->SetInputConnection(arrowGlyph->GetOutputPort());
  vtkSmartPointer<vtkActor> arrowActor = 
      vtkSmartPointer<vtkActor>::New();
  arrowActor->SetMapper(arrowMapper);
  graphLayoutView->GetRenderer()->AddActor(arrowActor);
  
  graphLayoutView->ResetCamera();
  graphLayoutView->Render();
  graphLayoutView->GetInteractor()->Start();
     
  return EXIT_SUCCESS;
}
