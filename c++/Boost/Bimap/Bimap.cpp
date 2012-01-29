#include <iostream>
#include <map>
#include <string>

#include <boost/bimap/bimap.hpp>
#include <boost/bimap/set_of.hpp>

int main()
{
  typedef boost::bimaps::bimap< boost::bimaps::set_of<std::string>, boost::bimaps::set_of<int> > bimap_type;
  bimap_type bm;
  typedef bimap_type::left_map LeftMapType;
  LeftMapType& leftMap = bm.left;
  typedef LeftMapType::value_type LeftValueType;
  typedef LeftMapType::const_iterator ConstLeftIterator;

  leftMap.insert( LeftValueType("one",1) );
  ConstLeftIterator leftIterator = leftMap.find("one");
  std::cout << leftIterator->first << " --> " << leftIterator->second << std::endl;

  /////////////////////
  typedef bimap_type::right_map RightMapType;
  RightMapType& rightMap = bm.right;
  typedef RightMapType::value_type RightValueType;
  typedef RightMapType::const_iterator ConstRightIterator;

  rightMap.insert( RightValueType(2,"two") );
  ConstRightIterator rightIterator = rightMap.find(2);
  std::cout << rightIterator->first << " --> " << rightIterator->second << std::endl;;

  return 0;
}