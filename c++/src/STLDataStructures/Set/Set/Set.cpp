#include <iostream>
#include <set>
#include <vector>
#include <cstdlib>

#include <algorithm>

void TestSet();
void TestSetIsInserted();

int main(int argc, char* argv[])
{
  //TestSet();
  TestSetIsInserted();


  return 0;
}

void TestSet()
{
  // Create a set
  std::set<unsigned int> S;

  // Add 10 elements to the set
  for(unsigned int i = 0; i < 10; i++)
  {
    S.insert(i);
  }

  // Output all of the elements in the set
  for(std::set<unsigned int>::iterator it1 = S.begin(); it1 != S.end(); it1++)
  {
    std::cout << " " << *it1;
  }

  std::cout << "Size: " << S.size() << " Empty? " << S.empty() << std::endl;


}

void TestSetIsInserted()
{
  // Create a set
  std::set<unsigned int> S;

  typedef std::pair<std::set<unsigned int>::iterator,bool> ReturnType;

  // Add 10 elements to the set
  for(unsigned int i = 0; i < 10; i++)
    {
    ReturnType inserted = S.insert(i);
    if(inserted.second)
      {
      std::cout << "Inserted " << i << std::endl;
      }
    else
      {
      std::cout << "Did not insert " << i << std::endl;
      }
    }

    ReturnType inserted = S.insert(3);
    if(inserted.second)
      {
      std::cout << "Inserted " << 3 << std::endl;
      }
    else
      {
      std::cout << "Did not insert " << 3 << std::endl;
      }
}
