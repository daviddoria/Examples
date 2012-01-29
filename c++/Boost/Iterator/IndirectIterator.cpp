#include <iostream>
#include <set>

#include <boost/iterator/indirect_iterator.hpp>

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
    objects.insert(new int(3));
  }
  // Iterator interface
  //typedef boost::indirect_iterator<std::set<int> > iterator;
  typedef boost::indirect_iterator<std::set<int*>::iterator> iterator;

  iterator begin() { return objects.begin(); }

  iterator end() { return objects.end(); }

private:

  std::set<int*> objects;
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

