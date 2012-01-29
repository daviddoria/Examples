#include <iostream>
#include <queue>
#include <vector>
#include <cstdlib>
#include <algorithm>

float RandomNumber();

struct MyComparison
{
  bool operator()(const float v1, const float v2) const
  {
    if(v1 < v2)
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
  std::priority_queue <double, std::vector<double>, MyComparison > pq; // the first element will be the one with the smallest value

  for(unsigned int i = 0; i < 10; i++)
  {
    pq.push(RandomNumber());
  }

  std::cout<<"pq contains " << pq.size() << " elements.\n";

  while (!pq.empty())
  {
    std::cout << pq.top() << std::endl;   //print out the highest priority element
    pq.pop();                   //remove the highest priority element
  }

  return 0;
}

float RandomNumber()
{
  // Produce a random number between 0 and 1
  return drand48();
}
