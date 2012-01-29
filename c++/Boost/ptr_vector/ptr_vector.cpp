#include <iostream>

#include <boost/ptr_container/ptr_vector.hpp>

// Expose the itererator if the container stores objects.
// class MyCollection
// {
// public:
//   // Iterator interface
//   typedef std::set<int>::iterator iterator;
//   typedef std::set<int>::const_iterator const_iterator;
// 
//   iterator begin() { return objects.begin(); }
// 
//   iterator end() { return objects.end(); }
// 
// private:
// 
//   std::set<int> objects;
// };

// Expose the itererator if the container stores pointers.
class MyCollection
{
public:
  MyCollection()
  {
    objects.push_back(new int(3));
  }
  
  // Iterator interface
  typedef boost::ptr_vector<int>::iterator iterator;

  iterator begin() { return objects.begin(); }

  iterator end() { return objects.end(); }

private:

  boost::ptr_vector<int> objects;
};

int main()
{
  MyCollection myCollection;

  for(MyCollection::iterator iterator = myCollection.begin(); iterator != myCollection.end(); iterator++)
  {
    std::cout << " " << *iterator;
  }
  return 0;
}
