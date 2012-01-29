#include <iostream>
#include <set>
#include <memory> // shared_ptr

#include <boost/iterator/indirect_iterator.hpp>

class MyCollection
{
public:
  MyCollection()
  {
    objects.insert(std::shared_ptr<int>(new int(6)));
  }

  typedef boost::indirect_iterator<std::set<std::shared_ptr<int> >::iterator> iterator;

  iterator begin() { return objects.begin(); }

  iterator end() { return objects.end(); }

private:

  std::set<std::shared_ptr<int> > objects;

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
