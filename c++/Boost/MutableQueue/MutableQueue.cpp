#include <boost/pending/mutable_queue.hpp>

#include <iostream>

int main ()
{
  boost::mutable_queue<float> queue; // this does not work
  //boost::mutable_queue<float> queue();
  //queue.push(1.0f);
  return 0;
}
