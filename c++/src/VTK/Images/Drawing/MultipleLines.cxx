#include <vtkPNGWriter.h>
#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include <vtkImageCanvasSource2D.h>

#include <vector>

struct Point
{
  int x,y;
};

struct Line
{
  Point p1, p2;
};

int main ( int argc, char* argv[] )
{
  // Draw a circle
  vtkSmartPointer<vtkImageCanvasSource2D> drawing =
    vtkSmartPointer<vtkImageCanvasSource2D>::New();
  drawing->SetScalarTypeToUnsignedChar();
  drawing->SetNumberOfScalarComponents(1);
  int extent[6] = {0,100,0,100,0,0};
  drawing->SetExtent(extent);
  drawing->SetDrawColor(255.0, 0.0, 0.0);

  std::vector<Line> lines;

  // one line
  Point p1;
  p1.x = 0;
  p1.y = 0;

  Point p2;
  p2.x = 10;
  p2.y = 10;

  Line line;
  line.p1 = p1;
  line.p2 = p2;

  lines.push_back(line);

  // another line
  p1.x = 50;
  p1.y = 50;

  p2.x = 60;
  p2.y = 60;

  line.p1 = p1;
  line.p2 = p2;

  lines.push_back(line);

  for (int i = 0; i < lines.size(); i++)
    {
    drawing->FillTube(lines[i].p1.x, lines[i].p1.y, lines[i].p2.x, lines[i].p2.y, 1.0);
    }
  drawing->Update();

  vtkSmartPointer<vtkPNGWriter> writer =
    vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetFileName("test.png");
  writer->SetInputConnection(drawing->GetOutputPort());
  writer->Write();

  return EXIT_SUCCESS;
}
