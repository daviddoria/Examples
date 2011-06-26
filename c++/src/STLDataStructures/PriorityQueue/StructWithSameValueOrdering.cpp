#include <iostream>
#include <queue>
#include <vector>
#include <cstdlib>
#include <algorithm>

struct MyStruct
{
  int value;
  int otherValue;
};

struct MyComparison
{
  bool operator()(const MyStruct v1, const MyStruct v2) const
  {
    if(v1.value < v2.value)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
};


int main(int argc, char* argv[])
{
  std::priority_queue <MyStruct, std::vector<MyStruct>, MyComparison > pq; // the first element will be the one with the smallest value

  for(unsigned int i = 0; i < 10; i++)
  {
    MyStruct a;
    a.value = 2;
    a.otherValue = i;
    pq.push(a);
  }

  while (!pq.empty())
  {
    std::cout << pq.top().value << " " << pq.top().otherValue << std::endl;   //print out the highest priority element
    pq.pop();                   //remove the highest priority element
  }

  return 0;
}
