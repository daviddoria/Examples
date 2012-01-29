#include <iostream>
#include <queue>
#include <vector>
#include <cstdlib>
#include <algorithm>

double RandomDouble();

int main(int argc, char* argv[])
{
  std::priority_queue<std::pair<double, bool> > priorityQueue;

  for(unsigned int i = 0; i < 10; i++)
  {
    std::pair<double, bool> element(RandomDouble(), true);
    priorityQueue.push(element);
  }

  std::cout << "priorityQueue contains " << priorityQueue.size() << " elements.\n";

  while (!priorityQueue.empty())
  {
    std::cout << priorityQueue.top().first << std::endl;   //print out the highest priority element
    priorityQueue.pop();                   //remove the highest priority element
  }

  return 0;
}

double RandomDouble()
{
  //produce a random double between 0 and 1
  return drand48();
}
