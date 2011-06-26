#include <iostream>
#include <queue>
#include <vector>
#include <cstdlib>
#include <algorithm>

double RandomDouble();
void TestQueue();
void Find();

int main(int argc, char* argv[])
{
  TestQueue();
  Find();
  return 0;
}

void TestQueue()
{
  std::cout << "TestQueue" << std::endl << "-------------" << std::endl;
  std::queue <double> q;

  for(unsigned int i = 0; i < 10; i++)
  {
    q.push(RandomDouble());
  }

  std::cout<<"q contains " << q.size() << " elements." << std::endl;

  while (!q.empty())
  {
    std::cout << q.front() << std::endl;    //print out the first element in the queue
    q.pop();                      //remove the first element of the queue
  }
}

double RandomDouble()
{
  //produce a random double between 0 and 1
  return drand48();
}

void Find()
{
  
   std::deque <int> q;

  q.push_front(1);
  q.push_front(2);
  q.push_front(3);
  
  //5 should not be found
  {
  std::deque<int>::iterator pos = std::find( q.begin(), q.end(), 5);
  if ( pos == q.end())
    {
    std::cout << "not found" << std::endl;
    }
  else
    {
    std::cout << "found" << std::endl;
    }
  }
  
  //2 should not be found
  {
  std::deque<int>::iterator pos = std::find( q.begin(), q.end(), 2);
  if ( pos == q.end())
    {
    std::cout << "not found" << std::endl;
    }
  else
    {
    std::cout << "found" << std::endl;
    }
  }
}