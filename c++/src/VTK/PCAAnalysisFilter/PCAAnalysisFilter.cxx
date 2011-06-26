#include <vtkSmartPointer.h>
#include <vtkFloatArray.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPlaneSource.h>
#include <vtkPCAAnalysisFilter.h>

int main(int argc, char *argv[])
{
  //create a set of points
  vtkSmartPointer<vtkPlaneSource> planeSource = 
      vtkSmartPointer<vtkPlaneSource>::New();
  planeSource->SetNormal(1,2,3);
  planeSource->Update();
  
  {
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInputConnection(planeSource->GetOutputPort());
  writer->SetFileName("plane.vtp");
  writer->Write();
  }
  vtkSmartPointer<vtkPCAAnalysisFilter> pcaAnalysis = 
      vtkSmartPointer<vtkPCAAnalysisFilter>::New();
  pcaAnalysis->SetNumberOfInputs(1);
  pcaAnalysis->SetInput(0, planeSource->GetOutput());
  pcaAnalysis->Update();
        
  vtkFloatArray* evals = pcaAnalysis->GetEvals();
  
  cout << "There are " << evals->GetNumberOfComponents() << " components." << endl;
  cout << "There are " << evals->GetNumberOfTuples() << " evals." << endl;
  
  for(vtkIdType i = 0; i < evals->GetNumberOfTuples(); i++)
    {
    float eval[1];
    evals->GetTupleValue(i, eval);
    cout << "Eval " << i << " : " << eval[0] << endl;
    }
    
  vtkSmartPointer<vtkPolyData> output = vtkPolyData::SafeDownCast(pcaAnalysis->GetOutput());
  
  //get the normals to get the evecs
  
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInput(output);
  writer->SetFileName("output.vtp");
  writer->Write();
  
  return EXIT_SUCCESS;
}
