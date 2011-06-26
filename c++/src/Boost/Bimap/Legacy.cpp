#include <iostream>
#include <map>
#include <string>

#include <boost/bimap/bimap.hpp>
#include <boost/bimap/set_of.hpp>

template< class Map, class CompatibleKey, class CompatibleData >
void use_it( Map & m,
             const CompatibleKey  & key,
             const CompatibleData & data )
{
    typedef typename Map::value_type value_type;
    typedef typename Map::const_iterator const_iterator;

    m.insert( value_type(key,data) );
    const_iterator iter = m.find(key);
    if( iter != m.end() )
    {
        std::cout << iter->first << " --> " << iter->second;
    }
    //m.erase(key);
}

int main()
{
  typedef boost::bimaps::bimap< boost::bimaps::set_of<std::string>, boost::bimaps::set_of<int> > bimap_type;
  bimap_type bm;
  typedef bimap_type::left_map map_type;
  map_type & m = bm.left;

  use_it( m, "one", 1 );

  std::cout << std::endl;
  typedef bimap_type::right_map reverse_map_type;
  reverse_map_type & rm = bm.right;

  use_it( rm, 1, "one" );

  return 0;
}