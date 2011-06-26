#include <set>

#include "Comparison.h"

int main(int, char*[])
{
  std::set<double, Comparison > s;
  s.insert(1.0);

  return 0;
}
