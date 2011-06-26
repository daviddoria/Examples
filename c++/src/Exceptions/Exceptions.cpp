#include <iostream>
#include <fstream>

using namespace std;

int main()
{
  try
  {
    // Statements that may throw exceptions you want to handle now go here
    throw -1;
  }
  catch (int)
  {
    // Any exceptions of type int thrown within the above try block get sent here
    cerr << "We caught an exception of type int" << endl;
  }
  catch (double)
  {
    // Any exceptions of type double thrown within the above try block get sent here
    cerr << "We caught an exception of type double" << endl;
  }
  catch (...)
  {
    cerr << "We caught an exception of unknown type." << endl;
  }

  return 0;
}
