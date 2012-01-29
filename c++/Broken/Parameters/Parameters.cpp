#include <iostream>
#include <vector>
#include <cstdlib>
#include <boost/parameter/name.hpp>

BOOST_PARAMETER_FUNCTION(
	(bool),                // 1. parenthesized return type
	Test,    // 2. name of the function template
	tag,                   // 3. namespace of tag types
	(required (a, *) ) // 4. one required parameter, and
	(optional              //    four optional parameters, with defaults
	(b,           *, )
	(c,       *, *)
	
	)
)
{
      // ... body of function goes here...
      // use graph, visitor, index_map, and color_map
}


int main(int argc, char* argv[])
{
	
	return 0;
}
