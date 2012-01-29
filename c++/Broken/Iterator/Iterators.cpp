#include <iostream>
#include <vector>
#include <map>

void Input();
void Output();

void AllMapElements();

int main(int argc, char *argv[])
{
  //Input();
  //Output();

  AllMapElements();
  return 0;
}

void Input()
{
  std::vector<double> a;
  a.push_back(1.2);
  a.push_back(1.3);
  a.push_back(1.4);
  a.push_back(1.5);
  a.push_back(1.6);

  std::vector<double>::iterator i;

  i = a.begin();

  while( i != a.end() )
  {
    std::cout << *i << std::endl;
    i++;
  }
}

void Output()
{

  std::vector<double> a(5);

  std::vector<double>::iterator i;

  i = a.begin();

  int counter = 0;
  while( i != a.end() )
  {
    *i = counter;
    counter++;
    i++;
  }

  i = a.begin();

  while( i != a.end() )
  {
    std::cout << *i << std::endl;
    i++;
  }
}

void AllMapElements()
{
  std::map <double, int> MyMap;
  MyMap[56.7] = 5;
  MyMap[99.3] = 9;
  MyMap[12.9] = 1;

  std::map <double, int>::iterator i;

  i = MyMap.begin();
	
  while( i != MyMap.end() )
  {
    std::cout << i->first << std::endl;
    std::cout << i->second << std::endl;
    std::cout << std::endl;
    i++;
  }
}