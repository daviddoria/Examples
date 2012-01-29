#include <iostream>
#include <set>

// See instead boost::indirect_iterator
class MyIterator
{
public:
  typedef std::set<int>::iterator iterator;
  typedef std::set<int>::const_iterator const_iterator;

  iterator begin() { return objects.begin(); }

  iterator end() { return objects.end(); }

private:
  std::set<int*> objects;

};

int main(int argc, char *argv[])
{
  MyIterator myIterator;
  for(MyIterator::iterator iterator = myIterator.begin(); iterator != myIterator.end(); iterator++)
  {
    std::cout << " " << *iterator;
  }
  return 0;
}
