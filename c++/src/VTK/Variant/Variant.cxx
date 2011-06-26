#include <vtkVariant.h>

int main(int argc, char *argv[])
{
  double dVal = vtkVariant("2").ToDouble();
  cout << "dVal: " << dVal << endl;
  vtkstd::string strVal = vtkVariant(dVal).ToString();
  cout << "strVal: " << strVal << endl;
  return EXIT_SUCCESS;
}
