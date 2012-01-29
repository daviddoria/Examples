#include <iostream>
#include <vector>

#include "ParallelSort.h"

int main (int argc, char *argv[]) 
{
  std::vector<int> v;
  v.push_back(4);
  v.push_back(1);
  v.push_back(7);

  ParallelSort::OutputVector<int>(v);
  
  std::vector<ParallelSort::IndexedValue<int> > sorted = ParallelSort::ParallelSort<int>(v);
  
  ParallelSort::OutputIndexedValueVector<ParallelSort::IndexedValue<int> >(sorted);
  
  return 0;
}
