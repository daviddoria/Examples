#include <vtkSmartPointer.h>
#include <vtkLookupTable.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkLookupTable> lookupTable = 
      vtkSmartPointer<vtkLookupTable>::New();
  
  lookupTable->SetTableRange(0.0, 10.0);
  //if you don't want to use the whole color range, you can use
  //SetValueRange, SetHueRange, and SetSaturationRange
  lookupTable->Build();
  
  double color[3];
  lookupTable->GetColor(1.0, color);
  cout << color[0] << " " << color[1] << " " << color[2] << endl;
  
  lookupTable->GetColor(5.0, color);
  cout << color[0] << " " << color[1] << " " << color[2] << endl;
  
  return EXIT_SUCCESS;
}
