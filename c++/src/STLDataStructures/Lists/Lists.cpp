#include <iostream>
#include <list>
#include <vector>
#include <cstdlib>

using namespace std;

double RandomDouble();
void Report(list<double> &L);
void Output(list<double> &L);
void Output(vector<double> &V);
		
vector<double> ListToVector(const list<double> &L);
list<double> VectorToList(const vector<double> &V);

void TestList();
void TestListToVector();
void TestVectorToList();

void TestRemoveElement();

int main(int argc, char* argv[])
{
  //TestList();
  
  //TestListToVector();
  //TestVectorToList();
  
  TestRemoveElement();
  
  return 0;
}

void TestRemoveElement()
{
  list<double> L;

  for(unsigned int i = 0; i < 10; i++)
    {
    L.push_back(i);
    }
    
    
  for(list<double>::iterator it1 = L.begin(); it1 != L.end(); it1++)
    {
    cout << " " << *it1;
    }
  std::cout << std::endl;
    
  list<double>::iterator it1 = L.begin();
  advance(it1, 1);
  advance(it1, 1);
  L.erase(it1);
  
  for(list<double>::iterator it1 = L.begin(); it1 != L.end(); it1++)
    {
    cout << " " << *it1;
    }
}

void TestList()
{
  list<double> L;

  Report(L);
  for(unsigned int i = 0; i < 10; i++)
    {
    L.push_back(RandomDouble());
    }
  
  Report(L);
  Output(L);

  //erase the second element
  list<double>::iterator it1 = L.begin();
  it1++;
  L.erase(it1);
  
  Report(L);
  Output(L);
  
  L.clear();
  
  Report(L);

}

void TestListToVector()
{
  list<double> L;

  for(unsigned int i = 0; i < 10; i++)
	  L.push_back(RandomDouble());
  
  Output(L);
  
  vector<double> V = ListToVector(L);
  
  Output(V);
}

void TestVectorToList()
{
  vector<double> V;

  for(unsigned int i = 0; i < 10; i++)
	  V.push_back(RandomDouble());
  
  Output(V);
  
  list<double> L = VectorToList(V);
  
  Output(L);
}

double RandomDouble()
{
  //produce a random double between 0 and 1
  return drand48();
}

void Output(list<double> &L)
{
  //for(list<double>::iterator it1 = L.begin(); it1 != L.end(); ++it1)
  for(list<double>::iterator it1 = L.begin(); it1 != L.end(); it1++)
  {
    cout << " " << *it1;
  }
  
  cout << endl;
}

void Output(vector<double> &V)
{
  for(unsigned int i = 0; i < V.size(); i++)
  {
    cout << " " << V[i];
  }
  
  cout << endl;
}

void Report(list<double> &L)
{
  std::cout << "Size: " << L.size() << " Empty? " << L.empty() << std::endl;
}

vector<double> ListToVector(const list<double> &L)
{
  vector<double> V(L.begin(), L.end());
  return V;
}

list<double> VectorToList(const vector<double> &V)
{
  list<double> L(V.begin(), V.end());
  return L;
}
