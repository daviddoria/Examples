#include <iostream>
#include <vector>
#include <set>

template<typename T>
void Output(T& myObject)
{
  for(typename T::iterator it1 = myObject.begin(); it1 != myObject.end(); it1++)
  {
    std::cout << *it1 << " ";
  }

  std::cout << std::endl;
}


int main(int, char *[])
{
  std::vector<double> myVector;

  myVector.push_back(1.1);
  myVector.push_back(2.2);

  Output(myVector);

  std::set<double> mySet;

  mySet.insert(1.1);
  mySet.insert(2.2);

  Output(mySet);

  return 0;
}

