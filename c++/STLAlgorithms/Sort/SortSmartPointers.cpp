#include <iostream>
#include <vector>
#include <algorithm>
#include <memory>

struct Test
{
public:
  Test(const unsigned int input) : a(input){}
  int a;
};

struct SortFunctor
{
  bool operator()(const std::shared_ptr<Test>& object1, const std::shared_ptr<Test>& object2)
  //bool operator()(std::shared_ptr<Test>& object1, std::shared_ptr<Test>& object2)
  //bool operator()(std::shared_ptr<Test> object1, std::shared_ptr<Test> object2)
  {
    return(object1->a < object2->a);
  }
};

//////////////////////////
int main (int argc, char *argv[])
{
  srand(time(NULL));

  std::vector<std::shared_ptr<Test> > objects;
  std::shared_ptr<Test> object1(new Test(rand()%50));
  objects.push_back(object1);

  std::shared_ptr<Test> object2(new Test(rand()%50));
  objects.push_back(object2);

  std::sort(objects.begin(), objects.end(), SortFunctor());

  return 0;
}
