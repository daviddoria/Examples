#include <iostream>
#include <set>
#include <cmath>

// Don't allow elements that are within 'tolerance' of each other to be added to the set
struct MyComparison
{
  MyComparison(float tol) : tolerance(tol){}
  
  float tolerance;
  
  bool operator()(const float s1, const float s2) const
  {

    if((s1 < s2) && !( fabs(s1 - s2) < tolerance))
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
  MyComparison myComparison(1e-4);
  std::set<float, MyComparison> S(myComparison);

  // Add 10 integers (0 to 9) to the set
  for(unsigned int i = 0; i < 10; i++)
  {
    S.insert(static_cast<float>(i));
  }

  // Output all of the elements in the set
  for(std::set<float>::iterator it1 = S.begin(); it1 != S.end(); it1++)
  {
    std::cout << " " << *it1;
  }
  return 0;
}
