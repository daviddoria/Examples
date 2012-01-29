#include <iostream>

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
    std::cerr << "We caught an exception of type int" << std::endl;
  }
  catch (double)
  {
    // Any exceptions of type double thrown within the above try block get sent here
    std::cerr << "We caught an exception of type double" << std::endl;
  }
  catch (char*)
  {
    // Any exceptions of type double thrown within the above try block get sent here
    std::cerr << "We caught an exception of type double" << std::endl;
  }
  catch (...)
  {
    std::cerr << "We caught an exception of unknown type." << std::endl;
  }

  return 0;
}
