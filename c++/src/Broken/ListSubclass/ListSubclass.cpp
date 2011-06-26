#include <iostream>
#include <list>

#include <iostream>
#include <list>

using std::list;

template <class T>
class leda_list : public list<T>
{
  /* Ryan Boller's additions to make a LEDA-like interface */
private:
  //iterator loopIter;
  //iterator ledaIter, tmpCursor;
  typename list<T>::Iterator loopIter;
};

int main (int argc, char *argv[]) 
{

	return 0;
}

