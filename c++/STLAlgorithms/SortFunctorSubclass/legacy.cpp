#include <iostream>
#include <vector>
#include <algorithm>

struct Test
{
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
    return(T1.a < T2.a);
  }
};

//////////////////////////
int main (int argc, char *argv[])
{
  std::vector<Test> testVector = CreateVector();

  std::cout << "unsorted:" << std::endl;
  for(unsigned int i = 0; i < testVector.size(); i++)
  {
    std::cout << testVector[i].a << " " << testVector[i].b << std::endl;
  }

//   SortFunctorSubclass test;
//   std::sort(testVector.begin(), testVector.end(), test);
//   std::cout << "sorted:" << std::endl;
//   for(unsigned int i = 0; i < testVector.size(); i++)
//   {
//     std::cout << testVector[i].a << " " << testVector[i].b << std::endl;
//   }

  SortFunctor* test2 = new SortFunctorSubclass;

  std::sort<std::vector<Test>::iterator, SortFunctor const&>(testVector.begin(), testVector.end(), static_cast<SortFunctor&>(*test2));
  //f<B const&>(static_cast<B&>(d));
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
  Test t;
  t.a = rand()%50;
  t.b = rand()%50;
  testVector.push_back(t);

  t.a = rand()%50;
  t.b = rand()%50;
  testVector.push_back(t);

  t.a = rand()%50;
  t.b = rand()%50;
  testVector.push_back(t);

  return testVector;
}
