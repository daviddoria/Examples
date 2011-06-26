#include <iostream>
#include <vector>
#include <algorithm>

void Standard();
void Descending();
    
void NonStandard();

struct Test
{
  int a,b;
};

struct SortFunctor
{
  bool operator()(const Test &T1, const Test &T2)
  {
    return(T1.a < T2.a);
  }
};

bool operator<(const Test &T1, const Test &T2)
{
  return(T1.a < T2.a);
}

bool SortB(const Test &T1, const Test &T2)
{
  return(T1.b < T2.b);
}

//////////////////////////
int main (int argc, char *argv[]) 
{
  srand(time(NULL));
  //Standard();
  Descending();
  //NonStandard();

  return 0;
}

void Standard()
{
  std::vector<double> a;
  a.push_back(4.5);
  a.push_back(1.2);
  a.push_back(7.8);

  //sort in ascending order
  std::sort(a.begin(), a.end());

  for(int i = 0; i < a.size(); i++)
  {
    std::cout << a[i] << std::endl;
  }
}

void Descending()
{
  std::vector<double> a;
  a.push_back(4.5);
  a.push_back(1.2);
  a.push_back(7.8);

  //sort in ascending order
  std::sort(a.rbegin(), a.rend());

  for(int i = 0; i < a.size(); i++)
  {
    std::cout << a[i] << std::endl;
  }
}

void NonStandard()
{
  std::vector<Test> T;
  Test T1;
  T1.a = rand()%50;
  T1.b = rand()%50;
  Test T2;
  T2.a = rand()%50;
  T2.b = rand()%50;
  Test T3;
  T3.a = rand()%50;
  T3.b = rand()%50;
  T.push_back(T1);
  T.push_back(T2);
  T.push_back(T3);

  std::sort(T.begin(), T.end());

  std::cout << "Sort by a:" << std::endl;
  for(unsigned int i = 0; i < T.size(); i++)
  {
    std::cout << T[i].a << " " << T[i].b << std::endl;
  }
  
  std::sort(T.begin(), T.end(), SortB);

  std::cout << std::endl << "Sort by b:" << std::endl;
  for(unsigned int i = 0; i < T.size(); i++)
  {
    std::cout << T[i].a << " " << T[i].b << std::endl;
  }

  std::sort(T.begin(), T.end(), SortFunctor());
}