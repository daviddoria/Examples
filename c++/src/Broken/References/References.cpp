#include <iostream>

using namespace std;

class bar
{
  double d;
};

class foo
{
  bar b;
public:
  bar getb();
};

int main()
{
  return 0;
}
