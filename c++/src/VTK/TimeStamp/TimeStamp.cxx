#include <vtkTimeStamp.h>

int main(int argc, char *argv[])
{

  vtkTimeStamp timeStamp;
  cout << "Time stamp: " << timeStamp << endl;
  timeStamp.Modified();
  cout << "Time stamp: " << timeStamp << endl;

  return EXIT_SUCCESS;
}
