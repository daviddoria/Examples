#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkSelection.h>
#include <vtkSelectionNode.h>
#include <vtkPointData.h>
#include <vtkSphereSource.h>
#include <vtkFloatArray.h>
#include <vtkSelectEnclosedPoints.h>
#include <vtkExtractSelectedThresholds.h>
#include <vtkIntArray.h>
#include <vtkDataArray.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkXMLPolyDataWriter.h>
 
int main(int argc, char **argv)
{ 
  vtkSmartPointer<vtkSphereSource> sphereSource1 = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource1->SetCenter(0.0, 0.0, 0.0);
  sphereSource1->SetRadius(1.0);
  sphereSource1->Update();
  
    { 
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName("sphere1.vtp");
  writer->SetInputConnection(sphereSource1->GetOutputPort());
  writer->Write();
  }
  
  vtkSmartPointer<vtkSphereSource> sphereSource2 = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource2->SetCenter(1.0, 0.0, 0.0);
  sphereSource2->SetRadius(1.0);
  sphereSource2->Update();
    { 
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName("sphere2.vtp");
  writer->SetInputConnection(sphereSource2->GetOutputPort());
  writer->Write();
  }
  
  vtkSmartPointer<vtkSelectEnclosedPoints> selectEnclosedPoints = 
      vtkSmartPointer<vtkSelectEnclosedPoints>::New();
  selectEnclosedPoints->SetInput(sphereSource1->GetOutput());
  selectEnclosedPoints->SetSurface(sphereSource2->GetOutput());
  selectEnclosedPoints->Update();
   
  { 
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName("enclosed.vtp");
  writer->SetInputConnection(selectEnclosedPoints->GetOutputPort());
  writer->Write();
  }

  vtkSmartPointer<vtkFloatArray> thresh =
      vtkSmartPointer<vtkFloatArray>::New();
  thresh->SetNumberOfComponents(1);
  thresh->InsertNextValue(0.5);
  thresh->InsertNextValue(2.0);
  thresh->SetName("SelectedPoints");
  
  vtkSmartPointer<vtkSelection> selection = 
      vtkSmartPointer<vtkSelection>::New();
  vtkSmartPointer<vtkSelectionNode> selectionNode = 
      vtkSmartPointer<vtkSelectionNode>::New();
  selectionNode->SetContentType(vtkSelectionNode::THRESHOLDS);
  selectionNode->SetFieldType(vtkSelectionNode::POINT);
  selectionNode->SetSelectionList(thresh);
  selection->AddNode(selectionNode);

  vtkSmartPointer<vtkExtractSelectedThresholds> extract = 
      vtkSmartPointer<vtkExtractSelectedThresholds>::New();
  extract->SetInput(0, selectEnclosedPoints->GetOutput());
  extract->SetInput(1, selection);
  extract->SetInputArrayToProcess(0,0,0,
    vtkDataObject::FIELD_ASSOCIATION_POINTS, "SelectedPoints");
  extract->Update(); //outputs a vtkUnstructuredGrid
  
  vtkSmartPointer<vtkXMLUnstructuredGridWriter> writer = 
      vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
  writer->SetFileName("output.vtu");
  writer->SetInputConnection(extract->GetOutputPort());
  writer->Write();
  
  return EXIT_SUCCESS;
}
