#include <iostream>
#include <list>
#include <algorithm>
#include <string>
#include <iterator>

template<class Coll>
void print(const Coll c)
{
  typename Coll::const_iterator itr = c.begin();
  std::cout << "{ ";
  while(itr != c.end())
  {
    std::cout << *itr++ << " ";
  }
  std::cout << "}";
}

int main()
{
  using namespace std;
  string s1 = "abc";
  string s2 = "abd";
  list<char> a = list<char>(s1.begin(),s1.end());
  list<char> b = list<char>(s2.begin(),s2.end());
  list<char> res;

  std::set_difference(a.begin(),a.end(),b.begin(),b.end(),std::back_insert_iterator<list<char> >(res));
  print(a);
  std::cout << " - ";
  print(b);
  std::cout << " = ";
  print(res);
  std::cout << endl;

  return 0;
}