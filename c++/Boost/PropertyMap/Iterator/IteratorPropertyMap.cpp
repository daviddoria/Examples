#include <iostream>
#include <map>
#include <string>
#include <boost/property_map/property_map.hpp>

template <typename AddressMap>
void foo(AddressMap address)
{
  typedef typename boost::property_traits<AddressMap>::value_type value_type;
  typedef typename boost::property_traits<AddressMap>::key_type key_type;

  value_type old_address, new_address;
  key_type fred = "Fred";
  old_address = get(address, fred);
  new_address = "384 Fitzpatrick Street";
  put(address, fred, new_address);

  key_type joe = "Joe";
  value_type& joes_address = address[joe];
  joes_address = "325 Cushing Avenue";
}

int main()
{
  typedef std::map<std::string, std::string> StringToStringMapType;
  StringToStringMapType name2address;

  name2address.insert(make_pair(std::string("Fred"),
                                std::string("710 West 13th Street")));
  name2address.insert(make_pair(std::string("Joe"),
                                std::string("710 West 13th Street")));

  boost::associative_property_map<StringToStringMapType> address_map(name2address);
  foo(address_map);

  for (StringToStringMapType::iterator i = name2address.begin();
       i != name2address.end(); ++i)
  {
    std::cout << i->first << ": " << i->second << "\n";
  }

  return EXIT_SUCCESS;
}
