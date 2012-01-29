#include <iostream>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

// See http://www.boost.org/doc/libs/1_40_0/libs/property_map/doc/dynamic_property_map.html

void manipulate_freds_info(boost::dynamic_properties& properties)
{
  using boost::get;
  std::string fred = "Fred";

  int old_age = get<int>("age", properties, fred);
  std::string old_gpa = get("gpa", properties, fred);

  std::cout << "Fred's old age: " << old_age << "\n"
            << "Fred's old gpa: " << old_gpa << "\n";

  std::string new_age = "18";
  double new_gpa = 3.9;
  put("age",properties,fred,new_age);
  put("gpa",properties,fred,new_gpa);
}

int main()
{
  using boost::get;

  // build property maps using associative_property_map
  std::map<std::string, int> name2age;
  std::map<std::string, double> name2gpa;
  boost::associative_property_map< std::map<std::string, int> >
    age_map(name2age);
  boost::associative_property_map< std::map<std::string, double> >
    gpa_map(name2gpa);

  std::string fred("Fred");
  // add key-value information
  name2age.insert(make_pair(fred,17));
  name2gpa.insert(make_pair(fred,2.7));

  // build and populate dynamic interface
  boost::dynamic_properties properties;
  properties.property("age",age_map);
  properties.property("gpa",gpa_map);

  manipulate_freds_info(properties);

  std::cout << "Fred's age: " << get(age_map,fred) << "\n"
            << "Fred's gpa: " << get(gpa_map,fred) << "\n";       
}
