#include <iostream>
#include <map>

using std::map;
using std::less;
using std::pair;

template <class Key, class T, class Compare = less<Key> >
class leda_map : public map<Key, T, Compare>
{
public:
  typename map<Key, T, Compare>::iterator iter;
  
    typename map<Key, T, Compare>::data_type& inf(typename map<Key, T, Compare>::iterator x) {
    return (*((insert(typename map<Key, T, Compare>::value_type((*x).first, T()))).first)).second;
  }
};

int main (int argc, char *argv[]) 
{

	return 0;
}

