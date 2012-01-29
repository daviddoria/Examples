#include <unordered_map>
#include <iostream>
#include <typeindex>
#include <functional>
#include <vector>

struct image_base { virtual ~image_base() {} /* ... */ } ;

template< typename T > struct image :  image_base 
{
  int get_it() { return sizeof(T) ; } 
};

template< typename T > int get_it( image_base* p ) 
{
  return dynamic_cast< image<T>* >(p)->get_it() ; 
}

std::unordered_map< std::type_index, std::function< int(image_base*) > > lookup =
{
    { typeid( image<int> ), get_it<int> },
    { typeid( image<double> ), get_it<double> }
    // etc
};
 
int main()
{
    std::vector< image_base* > seq = { new image<int>(), new image<double>(), new image<int>() } ;
    for( auto& p : seq ) 
    {
      std::cout << lookup[ typeid(*p) ](p) << '\n' ;
    }
}
