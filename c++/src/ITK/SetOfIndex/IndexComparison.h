#ifndef IndexComparison_h
#define IndexComparison_h

#include "itkIndex.h"

struct IndexComparison
{
  // If z(x, y) is true, then x is less than y
  // If z(y, x) is true, then y is less than x
  bool operator()(const itk::Index<2> item1, const itk::Index<2> item2) const
  {
    if((item1[0] < item2[0]) && (item1[1] < item2[1]))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
};

#endif