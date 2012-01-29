#include <iostream>

#include <stdexcept>

static void TestFunction();

int main()
{
  try
  {
    TestFunction();
  }
  catch (std::runtime_error e)
  {
    std::cerr << e.what() << std::endl;
  }
  catch (std::exception e) // This will catch the runtime_error if runtime_error isn't handled explicitly. However, the what() will always be "std::exception"
  {
    std::cerr << e.what() << std::endl;
  }
  
  return 0;
}

void TestFunction()
{
  throw std::runtime_error("Oops!");
}
