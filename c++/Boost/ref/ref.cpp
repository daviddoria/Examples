// This doesn't work because ref doesn't have operator()

#include <iostream>
#include <vector>
#include <algorithm>

#include <boost/ref.hpp>

struct Test
{
  int a,b;
};

std::vector<Test> CreateVector();

struct SortFunctor
{
  //virtual bool operator()(const Test &T1, const Test &T2) = 0;
  virtual bool operator()(const Test &T1, const Test &T2){};
};

struct SortFunctorSubclass : public SortFunctor
{
  bool operator()(const Test &T1, const Test &T2)
  {
    std::cout << "subclass" << std::endl;
    return(T1.a < T2.a);
  }
};

//////////////////////////
int main (int argc, char *argv[])
{
  std::vector<Test> testVector = CreateVector();

  SortFunctor* test2 = new SortFunctorSubclass;

  std::sort(testVector.begin(), testVector.end(), boost::ref(*test2));

  return 0;
}

std::vector<Test> CreateVector()
{
  srand(time(NULL));
  std::vector<Test> testVector;

  return testVector;
}
