#ifndef Comparison_H
#define Comparison_H

struct Comparison
{
  bool operator()(const double item1, const double item2) const
  {
    if(item1 < item2)
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