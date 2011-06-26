#include <vtkSmartPointer.h>
#include <vtkVector.h>

int main(int argc, char *argv[])
{
  vtkColor3d color3d;
  color3d.SetRed(.5);
  color3d.SetGreen(.6);
  color3d.SetBlue(.7);
  
  cout << color3d.Red() << " " << color3d.Green() << " " << color3d.Blue() << endl;
  
  vtkColor3ub color3ub;
  color3ub.SetRed(100);
  color3ub.SetGreen(105);
  color3ub.SetBlue(120);
  
  cout << int(color3ub.Red()) << " " << int(color3ub.Green()) << " " << int(color3ub.Blue()) << endl;
  
  return EXIT_SUCCESS;
}
