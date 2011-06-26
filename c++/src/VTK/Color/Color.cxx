#include <vtkSmartPointer.h>
#include <vtkColor.h>

int main(int argc, char *argv[])
{
  vtkColor3ub color;
  color.SetRed(255);
  color.SetGreen(10);
  color.SetBlue(0);

  cout << "Color: " << (int)color.GetRed() << " " << (int)color.GetGreen() << " " << (int)color.GetBlue() << endl;
  
  unsigned char* c = color.GetData();
  
  cout << "Color: " << (int)c[0] << " " << (int)c[1] << " " << (int)c[2] << endl;
  
  return EXIT_SUCCESS;
}
