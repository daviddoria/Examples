// This doesn't work because shared_ptr doesn't have operator()

#include <iostream>
#include <vector>

#include <boost/shared_ptr.hpp>

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

int main(int, char* [])
{
  boost::shared_ptr<SortFunctor> testPointer(new SortFunctorSubclass);

  std::vector<Test> testVector = CreateVector();

  std::sort(testVector.begin(), testVector.end(), testPointer);

  return 0;
}


std::vector<Test> CreateVector()
{
  srand(time(NULL));
  std::vector<Test> testVector;

  return testVector;
}
