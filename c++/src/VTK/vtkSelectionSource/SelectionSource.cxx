#include <vtkSmartPointer.h>
#include <vtkPointSource.h>
#include <vtkExtractSelection.h>
//#include <vtkSelection.h>
#include <vtkSelectionNode.h>
#include <vtkSelectionSource.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>
#include <vtkIdTypeArray.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPointSource> pointSource = 
      vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetNumberOfPoints(50);
  pointSource->Update();
  
  cout << "There are " << pointSource->GetOutput()->GetNumberOfPoints() << " input points." << endl;
  
  vtkSmartPointer<vtkSelectionSource> selectionSource = 
      vtkSmartPointer<vtkSelectionSource>::New();
  selectionSource->SetContentType(vtkSelectionNode::INDICES);
  selectionSource->SetFieldType(vtkSelectionNode::POINT);
  
  for (int i = 10; i <= 20; i++)
    {
    selectionSource->AddID(-1, i);
    }
  
  selectionSource->Update();
        
  vtkSmartPointer<vtkExtractSelection> extractSelection = 
      vtkSmartPointer<vtkExtractSelection>::New();
  
  extractSelection->SetInput(0, pointSource->GetOutput());
  extractSelection->SetInput(1, selectionSource->GetOutput());
  extractSelection->Update();
  
  vtkDataSet* ds = vtkDataSet::SafeDownCast (extractSelection->GetOutput());
  
  cout << "There are " << ds->GetNumberOfPoints() << " output points." << endl;
  
  return EXIT_SUCCESS;
}
