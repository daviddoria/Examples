#include <iostream>
#include <algorithm>
#include <vector>
#include <memory> // shared_ptr

#include <boost/iterator/indirect_iterator.hpp>

class MyClass
{
public:
  MyClass(const int a) : constMember(a){}
  const int constMember;

};

bool operator<(const MyClass &object1, const MyClass &object2)
{
  return(object1.constMember < object2.constMember);
}

class MyCollection
{
private:
  typedef std::vector<std::shared_ptr<MyClass> > ContainerType;

public:
  MyCollection()
  {
    objects.push_back(std::shared_ptr<MyClass>(new MyClass(6)));
  }

  typedef boost::indirect_iterator<ContainerType::iterator> iterator;

  iterator begin() { return objects.begin(); }

  iterator end() { return objects.end(); }

private:

  ContainerType objects;

};


int main()
{
  MyCollection myCollection;

  for(MyCollection::iterator iterator = myCollection.begin(); iterator != myCollection.end(); iterator++)
  {
    std::cout << " " << (*iterator).constMember;
  }

  sort(myCollection.begin(), myCollection.end());
  return 0;
}
