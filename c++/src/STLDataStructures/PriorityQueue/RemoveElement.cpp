#include <iostream>
#include <queue>
#include <vector>
#include <cstdlib>
#include <algorithm>

/* I don't think this is possible without popping all of the elements into another queue */

double RandomDouble();

int main(int argc, char* argv[])
{
  std::cout << "TestPriorityQueue" << std::endl << "-------------" << std::endl;
  std::priority_queue <double> pq;

  for(unsigned int i = 0; i < 10; i++)
  {
    pq.push(RandomDouble());
  }

  std::cout<<"pq contains " << pq.size() << " elements.\n";

  while (!pq.empty())
  {
    std::cout << pq.top() << std::endl;   //print out the highest priority element
    pq.pop();                   //remove the highest priority element
  }

  return 0;
}

double RandomDouble()
{
  //produce a random double between 0 and 1
  return drand48();
}
