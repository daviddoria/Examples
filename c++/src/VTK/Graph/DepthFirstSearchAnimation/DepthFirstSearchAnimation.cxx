#include <vtkSmartPointer.h>
#include <vtkRendererCollection.h>
#include <vtkIntArray.h>
#include <vtkLookupTable.h>
#include <vtkViewTheme.h>
#include <vtkDataSetAttributes.h>
#include <vtkUnsignedCharArray.h>
#include <vtkGraphLayoutView.h>
#include <vtkMutableDirectedGraph.h>
#include <vtkTree.h>
#include <vtkCommand.h>
#include <vtkTreeDFSIterator.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>

class vtkTimerCallback : public vtkCommand
{
  public:
    static vtkTimerCallback *New()
    {
      vtkTimerCallback *cb = new vtkTimerCallback;
      
      return cb;
    }
 
    virtual void Execute(vtkObject *caller, unsigned long eventId,
                         void *callData)
    {
      if (eventId != vtkCommand::TimerEvent)
        {
        return;
        }
        
      if(this->DFS->HasNext())
        {
        vtkIdType nextVertex = this->DFS->Next();
        //cout << "Next vertex: " << NextVertex << " level: " << tree->GetLevel(NextVertex) << endl;
        cout << "Next vertex: " << nextVertex << endl;
        //vertexColors->SetValue(nextVertex, 10);
        vtkIntArray::SafeDownCast(this->Tree->GetVertexData()->GetArray("color"))->SetValue(nextVertex, 10);
        this->Tree->Modified();
        this->GraphLayoutView->AddRepresentationFromInput(this->Tree);

        this->GraphLayoutView->Render();
  
        }

      }
    
  void SetDFS(vtkSmartPointer<vtkTreeDFSIterator> dfs) {this->DFS = dfs;}
  void SetTree(vtkSmartPointer<vtkTree> tree) {this->Tree = tree;}
  void SetGraphLayoutView(vtkSmartPointer<vtkGraphLayoutView> view) {this->GraphLayoutView = view;}
  
  private:
    
    vtkSmartPointer<vtkTreeDFSIterator> DFS;
    vtkSmartPointer<vtkTree> Tree;
    vtkSmartPointer<vtkGraphLayoutView> GraphLayoutView;
};

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkMutableDirectedGraph> graph = 
      vtkSmartPointer<vtkMutableDirectedGraph>::New();
  
  //create a tree
  vtkIdType v1 = graph->AddVertex();
  vtkIdType v2 = graph->AddChild(v1);
  vtkIdType v3 = graph->AddChild(v1);
  vtkIdType v4 = graph->AddChild(v2);
  
  vtkSmartPointer<vtkTree> tree = 
      vtkSmartPointer<vtkTree>::New();
  tree->CheckedShallowCopy(graph);
      
  //create the color array
  vtkSmartPointer<vtkIntArray> vertexColors = 
      vtkSmartPointer<vtkIntArray>::New();
  vertexColors->SetNumberOfComponents(1);
  vertexColors->SetName("color");
  
  
  vtkSmartPointer<vtkLookupTable> lookupTable = 
    vtkSmartPointer<vtkLookupTable>::New();
 
  lookupTable->SetTableRange(0.0, 10.0);
  lookupTable->Build();
  
  for(unsigned int i = 0; i < tree->GetNumberOfVertices(); i++)
    {
    vertexColors->InsertNextValue(0);
    }
    
  //add the color array to the tree
  tree->GetVertexData()->AddArray(vertexColors);

  //create a depth first search iterator
  vtkSmartPointer<vtkTreeDFSIterator> DFS = 
      vtkSmartPointer<vtkTreeDFSIterator>::New();
  vtkIdType root = tree->GetRoot();
  DFS->SetStartVertex(root);
  DFS->SetTree(tree);
        
  vtkSmartPointer<vtkGraphLayoutView> graphLayoutView = 
    vtkSmartPointer<vtkGraphLayoutView>::New();
  graphLayoutView->AddRepresentationFromInput(tree);
  graphLayoutView->SetLayoutStrategyToTree();
  
  graphLayoutView->SetVertexColorArrayName("color");
  graphLayoutView->ColorVerticesOn();
  
  vtkSmartPointer<vtkViewTheme> theme = 
      vtkSmartPointer<vtkViewTheme>::New();
  theme->SetPointLookupTable(lookupTable);
  theme->ScalePointLookupTableOff();
  
  graphLayoutView->ApplyViewTheme(theme);
  graphLayoutView->ResetCamera();
  graphLayoutView->Render();
  
    // Sign up to receive TimerEvent
  vtkSmartPointer<vtkTimerCallback> cb = 
      vtkSmartPointer<vtkTimerCallback>::New();
  cb->SetDFS(DFS);
  cb->SetTree(tree);
  cb->SetGraphLayoutView(graphLayoutView);
  
  int timerId = graphLayoutView->GetInteractor()->CreateRepeatingTimer(1000);
  graphLayoutView->GetInteractor()->AddObserver(vtkCommand::TimerEvent, cb);
  
  graphLayoutView->GetInteractor()->Start();
  
  return EXIT_SUCCESS;
}
