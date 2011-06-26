#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkTriangle.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkLine.h>
#include <vtkImageData.h>
#include <vtkProbeFilter.h>
#include <vtkDelaunay2D.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkDoubleArray.h>
#include <vtkLineSource.h>

int main ( int argc, char *argv[] )
{
  // Create two points, P0 and P1
  double p0[3] = {1.0, 0.0, 0.0};
  double p1[3] = {0.0, 1.0, 0.0};

  vtkSmartPointer<vtkLineSource> lineSource =
    vtkSmartPointer<vtkLineSource>::New();
  lineSource->SetPoint1(p0);
  lineSource->SetPoint2(p1);
  lineSource->Update();

  vtkSmartPointer<vtkPoints> probePoints =
    vtkSmartPointer<vtkPoints>::New();
  probePoints->InsertNextPoint(0, 0.5, 0);
  vtkSmartPointer<vtkPolyData> probePolyData =
    vtkSmartPointer<vtkPolyData>::New();
  probePolyData->SetPoints(probePoints);

  vtkSmartPointer<vtkProbeFilter> probe =
    vtkSmartPointer<vtkProbeFilter>::New();
  probe->SetInput(lineSource->GetOutput());
  probe->SetSource(probePolyData);
  probe->Update();

  vtkDataSet* output = probe->GetOutput();

  vtkDataArray* data = probe->GetOutput()->GetPointData()->GetScalars();
  if(data)
    {
    std::cout << "data is valid" << std::endl;
    }
  else
    {
    std::cout << "data is NOT valid" << std::endl;
    }

  vtkDoubleArray* doubleData = vtkDoubleArray::SafeDownCast (data);

  if(doubleData)
    {
    std::cout << "doubleData is valid" << std::endl;
    }

  for(unsigned int i = 0; i < doubleData->GetNumberOfTuples(); i++)
    {
    double val = doubleData->GetValue(i);
    }

  return EXIT_SUCCESS;
}