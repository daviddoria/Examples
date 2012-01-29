#include <iostream>
#include <stack>
#include <vector>
#include <cstdlib>
#include <algorithm>

double RandomDouble();

int main(int argc, char* argv[])
{
    std::cout << "TestStack" << std::endl << "-------------" << std::endl;
  std::stack <double> s;

  for(unsigned int i = 0; i < 10; i++)
  {
    s.push(RandomDouble());
  }

  std::cout << "s contains " << s.size() << " elements." << std::endl;

  while (!s.empty())
  {
    std::cout << s.top() << std::endl;    //print out the first element in the stack
    s.pop();                      //remove the first element of the stack
  }
  return 0;
}

double RandomDouble()
{
  //produce a random double between 0 and 1
  return drand48();
}
