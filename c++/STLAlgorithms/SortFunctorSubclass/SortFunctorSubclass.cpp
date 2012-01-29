#include <iostream>
#include <vector>
#include <algorithm>

struct Test
{
  Test(int a, int b) : a(a), b(b) {}
  int a,b;
};

std::vector<Test> CreateVector();

struct SortFunctor
{
  virtual bool operator()(const Test &T1, const Test &T2) = 0;
};

struct SortFunctorSubclass : public SortFunctor
{
  bool operator()(const Test &T1, const Test &T2)
  {
    std::cout << "subclass" << std::endl;
    return(T1.a < T2.a);
  }
};

struct SortFunctorWrapper
{
    SortFunctorWrapper(SortFunctor* func) : func_(func) {}
    bool operator()(const Test &T1, const Test &T2)
    {
        return (*func_)(T1, T2);
    }
    SortFunctor* func_;
};

int main (int argc, char *argv[]) 
{
  std::vector<Test> testVector = CreateVector();
  
  std::cout << "unsorted:" << std::endl;
  for(unsigned int i = 0; i < testVector.size(); i++)
  {
    std::cout << testVector[i].a << " " << testVector[i].b << std::endl;
  }

  SortFunctor* test2 = new SortFunctorSubclass;

  //std::sort<std::vector<Test>::iterator, SortFunctor const&>(testVector.begin(), testVector.end(), static_cast<SortFunctor&>(*test2));
  //std::sort(testVector.begin(), testVector.end(), static_cast<SortFunctor&>(*test2));
  //std::sort(testVector.begin(), testVector.end(), *test2);
  std::sort(testVector.begin(), testVector.end(), SortFunctorWrapper(test2));
  
  std::cout << "sorted:" << std::endl;
  for(unsigned int i = 0; i < testVector.size(); i++)
  {
    std::cout << testVector[i].a << " " << testVector[i].b << std::endl;
  }
  
  return 0;
}

std::vector<Test> CreateVector()
{
  srand(time(NULL));
  std::vector<Test> testVector;
  testVector.push_back(Test(1, 2));
  testVector.push_back(Test(3, 4));
  return testVector;
}
