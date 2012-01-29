#include <iostream>
#include <vector>
#include <cstdlib>

#include <algorithm> // max_element
#include <numeric> // accumulate()
#include <../../home/doriad/Test/Header1.h>

static double RandomDouble();
static void TestVector();
static void RemoveElement();
static void TestResize();
static void TestInsert();
static void TestCopy();
static void TestFind();
static void ArrayStyleInit();
static void Assign();
static void TestMax();
static void TestSum();
static void CopyPart();
static void RawData();

int main(int, char*[])
{
  //TestVector();
  //RemoveElement();
  //TestResize();
  //TestInsert();
  //TestCopy();
  //TestFind();
  //ArrayStyleInit();
  //Assign();
  //TestAppend();
  //TestMax();
  //TestSum();
  //CopyPart();
  RawData();
  return 0;
}

void CopyPart()
{
  std::vector<int> v(10);

  for(unsigned int i = 0; i < 10; i++)
  {
    v[i] = i;
  }
  
  std::vector<int> v2;
  v2.insert(v2.end(), v.begin(), v.begin() + 4);
  
  for(unsigned int i = 0; i < v2.size(); ++i)
    {
    std::cout << v2[i] << std::endl;
    }
}

void TestSum()
{
  std::vector<int> v(10);

  double manualSum = 0;
  for(unsigned int i = 0; i < 10; i++)
  {
    v[i] = 10*i;
    std::cout << v[i]   << " ";
    manualSum += v[i];
  }
  
  
  
  double sum = std::accumulate(v.begin(), v.end(), 0);
  
  std::cout << "sum: " << sum << std::endl;
  std::cout << "Manual sum: " << manualSum << std::endl;
}

void TestMax()
{
  std::vector<int> v(10);

  for(unsigned int i = 0; i < 10; i++)
  {
    v[i] = 10*i;
    std::cout << v[i]   << " ";
  }
  
  std::vector<int>::iterator it = std::max_element(v.begin(), v.end());
  unsigned int a = *it;
  std::cout << "Max value: " << a << " (position " << it - v.begin() << ")" << std::endl;
}

void ArrayStyleInit()
{
  //std::vector<int> vec({1,2,6,7,8,89}); //need -std=c++0x
}

void TestFind()
{
  std::vector<unsigned int> V(10);

  for(unsigned int i = 0; i < 10; i++)
  {
    V[i] = 10*i;
    std::cout << V[i]   << " ";
  }

  std::cout << std::endl;

  std::vector<unsigned int>::iterator it = find(V.begin(), V.end(), 70);

  if(it == V.end())
  {
      std::cout << "Could not find 70 in the vector"  << std::endl;
  }
  else
  {
    std::cout << "The number 70 is located at index " << it - V.begin() + 1 << std::endl; //should this have a +1?
  }
}

void TestCopy()
{
  std::vector<double> V(10);

  for(unsigned int i = 0; i < 10; i++)
  {
    V[i] = RandomDouble();
    std::cout << V[i]   << " ";
  }

  std::vector<double> B(V.size());

  std::copy(V.begin(), V.end(), B.begin());

  std::cout << std::endl;

  for(unsigned int i = 0; i < 10; i++)
  {
    std::cout << B[i]   << " ";
  }
}

double RandomDouble()
{
	//produce a random double between 0 and 1
	return drand48();
}

void TestResize()
{
	std::vector<double> V(10);

	for(unsigned int i = 0; i < 10; i++)
    {
		V[i] = RandomDouble();
    }

	for(unsigned int i = 0; i < 10; i++)
	{
		std::cout << V[i] << " ";
	}
	std::cout << "Size: " << V.size() << std::endl;

	V.resize(5);

	for(unsigned int i = 0; i < 5; i++)
	{
		std::cout << V[i] << " ";
	}
	std::cout << "Size: " << V.size() << std::endl;

}

void TestVector()
{
	std::vector<double> V;

	std::cout << "Size: " << V.size() << std::endl;
	std::cout << "Empty? " << V.empty() << std::endl;

	for(unsigned int i = 0; i < 100; i++)
    {
		V.push_back(RandomDouble());
    }

	std::cout << "Size: " << V.size() << std::endl;
	std::cout << "Empty? " << V.empty() << std::endl;

	V.clear();

	std::cout << "Size: " << V.size() << std::endl;
	std::cout << "Empty? " << V.empty() << std::endl;
}

void RemoveElement()
{
	unsigned int num = 10;
	std::vector<unsigned int> V(num);
	for(unsigned int i = 0; i < num; i++)
	{
		V[i] = i;
	}

	std::vector<unsigned int>::iterator it = find(V.begin(), V.end(), 7);
	if(it == V.end())
	{
		std::cout << "Could not find 7 in the vector"  << std::endl;
	}
	else
	{
		std::cout  << "Have found 7 in the vector"  << std::endl;
		std::cout << "The number 7 is located at index " << it - V.begin() << std::endl;
	}


	std::vector<unsigned int>::iterator it2 = find(V.begin(), V.end(), 15);
	if(it2 == V.end())
    {
		std::cout  << "Could not find 15 in the vector"  << std::endl;
    }
	else
    {
		std::cout  << "Have found 15 in the vector"  << std::endl;
    }

}

void TestInsert()
{
 std::vector <double> a;
 a.push_back(1);
 a.push_back(2);

 std::vector <double> b;
 b.push_back(3);
 b.push_back(4);

 std::vector<double> c;

 c.insert(c.end(), a.begin(), a.end());
 c.insert(c.end(), b.begin(), b.end());

 for(unsigned i = 0; i < c.size(); i++)
 {
     std::cout << c[i] << " ";
 }

 std::cout << std::endl;

}

void Assign()
{
  std::vector<int> myVec;

  myVec.push_back(2);

  myVec.assign(7,100);

  for(unsigned int i = 0; i < myVec.size(); i++)
    {
    std::cout << myVec[i] << std::endl;
    }
}

void RawData()
{
  std::vector<int> a;
  a.push_back(1);
  std::cout << a.data()[0] << std::endl;
}
