#include <iostream>
#include <queue>
#include <vector>
#include <cstdlib>
#include <algorithm>

double RandomDouble();

int main(int argc, char* argv[])
{
  std::cout << "TestPriorityQueue" << std::endl << "-------------" << std::endl;

  //std::priority_queue <double, std::vector<double>, std::less<double> > pq; // the first element will be the one with the largest value
  std::priority_queue <double, std::vector<double>, std::greater<double> > pq; // the first element will be the one with the smallest value

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
