#include <vtkSmartPointer.h>
#include <vtkPointSource.h>
#include <vtkExtractSelection.h>
#include <vtkSelectionSource.h>
#include <vtkSelectionNode.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPointSource> pointSource = 
      vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetNumberOfPoints(50);
  pointSource->Update();
  
  cout << "There are " << pointSource->GetOutput()->GetNumberOfPoints() << " input points." << endl;
  
  vtkSmartPointer<vtkSelectionSource> selectionSource = 
      vtkSmartPointer<vtkSelectionSource>::New();
  
  //selectionSource->AddThreshold(10, 20); //this is for thresholding scalar arrays
  for (int i = 10; i <= 20; i++)
  {
    selectionSource->AddID(-1, i);
  }
  selectionSource->Update();
  
  vtkSmartPointer<vtkExtractSelection> extractSelection = 
      vtkSmartPointer<vtkExtractSelection>::New();
  extractSelection->SetInput(0, pointSource->GetOutput());
  extractSelection->SetInput(1, selectionSource->GetOutput());
  selectionSource->SetContentType(vtkSelectionNode::INDICES);
  selectionSource->SetFieldType(vtkSelectionNode::POINT);
  extractSelection->Update();
  
  //vtkPolyData* extracted = extractSelection->GetOutput();
  //vtkUnstructuredGrid* extracted = extractSelection->GetOutput();
  
  vtkDataObject* obj = extractSelection->GetOutput();
  
  vtkDataSet* ds = vtkDataSet::SafeDownCast ( obj);
  
  cout << "There are " << ds->GetNumberOfPoints() << " output points." << endl;
  
  return EXIT_SUCCESS;
}
