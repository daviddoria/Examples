#include <iostream>
#include <vector>
#include <algorithm>
#include <string>

using namespace std;

void StructMethod();

struct NumberName
{
  int index;
  string Name;
};

bool operator<(NumberName NN1, NumberName NN2)
{
  return NN1.index < NN2.index;
}

int main (int argc, char *argv[]) 
{
  StructMethod();
  return 0;
}


void StructMethod()
{
  vector<string> Names;
  Names.push_back("Name1");
  Names.push_back("Name2");
  Names.push_back("Name3");

  vector<int> Ints;
  Ints.push_back(4);
  Ints.push_back(1);
  Ints.push_back(7);

  vector<NumberName> Pairs(Names.size());
  for(int i = 0; i < Names.size(); i++)
  {
	  Pairs[i].index = Ints[i];
	  Pairs[i].Name = Names[i];
  }

  sort(Pairs.begin(), Pairs.end());

  for(int i = 0; i < Pairs.size(); i++)
  {
	  cout << Pairs[i].index << " " << Pairs[i].Name << endl;
  }
}