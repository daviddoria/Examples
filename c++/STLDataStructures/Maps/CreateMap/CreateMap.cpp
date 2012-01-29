#include <iostream>
#include <map>
#include <string>
#include <algorithm> //max_element

static void InsertElement();
static void BadLookupElement();
static void GoodLookupElement();
static void OutputAll();
static void FindLargestValueFreeFunction();
static void FindLargestValueFunctor();

int main(int argc, char *argv[])
{
//   InsertElement();
//   BadLookupElement();
//   GoodLookupElement();
//   OutputAll();
//   FindLargestValueFreeFunction();
  FindLargestValueFunctor();
  return 0;
}

void InsertElement()
{
  // Create a map from strings to doubles
  std::map <std::string, double> myMap; // <key, value>
  std::cout << "original size: " << myMap.size() << std::endl;

  // Use this when you don't want the value to be default constructed before being assigned to if the entry doesn't exist.
  myMap.insert(std::pair<std::string, double>("One hundred eleven", 111));
  std::cout << "size after first insert: " << myMap.size() << std::endl;

  myMap["two"] = 2;
  std::cout << "size after second insert: " << myMap.size() << std::endl;

}

void BadLookupElement()
{
  // Create a map from strings to doubles
  std::map <std::string, double> myMap; // <key, value>
  myMap["one"] = 1;
  myMap["two"] = 2;

  // Key to value lookup: Method 1
  std::cout << myMap["one"] << std::endl;
  std::cout << myMap["two"] << std::endl;
  std::cout << "size before bad lookup: " << myMap.size() << std::endl;
  std::cout << myMap["three"] << std::endl; // this just outputs 0 without complaining. Even worse, it inserts a zero value with the specified key!
  std::cout << "size after bad lookup: " << myMap.size() << std::endl;
}

void GoodLookupElement()
{
  // Create a map from strings to doubles
  std::map <std::string, double> myMap; // <key, value>
  myMap["one"] = 1;
  myMap["two"] = 2;

  std::cout << myMap["one"] << std::endl;
  std::cout << myMap["two"] << std::endl;
  std::cout << "size before bad lookup: " << myMap.size() << std::endl;

  std::map<std::string, double>::iterator iter = myMap.find("three");

  if(iter == myMap.end())
  {
    std::cout << "Not found." << std::endl;
  }
  else
  {
    std::cout << "Found key " << iter->first << ". Associated value: " << iter->second << std::endl;
  }

  std::cout << "size after bad lookup: " << myMap.size() << std::endl;
}


void OutputAll()
{
  std::map <std::string, double> myMap; // <key, value>
  myMap["one"] = 1;
  myMap["two"] = 2;

  for(std::map<std::string, double>::const_iterator it = myMap.begin(); it != myMap.end(); it++)
    {
    std::string key = it->first;
    double value = it->second;
    std::cout << "key: " << key << " value: " << value << std::endl;
    }
}


typedef std::map <std::string, double> MapType;
bool value_compare_function(MapType::value_type &i1, MapType::value_type &i2)
{
  return i1.second < i2.second;
}

void FindLargestValueFreeFunction()
{
  std::cout << "FindLargestValue()" << std::endl;

  
  MapType myMap; // <key, value>
  myMap["one"] = 1;
  myMap["two"] = 2;


  MapType::iterator iterator = std::max_element(myMap.begin(), myMap.end(), value_compare_function);

  std::cout << iterator->first << " " << iterator->second << std::endl;
}

struct ValueCompareFunctor
{
  bool operator()(MapType::value_type &i1, MapType::value_type &i2)
  {
    return i1.second < i2.second;
  }
};

void FindLargestValueFunctor()
{
  std::cout << "FindLargestValue()" << std::endl;


  MapType myMap; // <key, value>
  myMap["one"] = 1;
  myMap["two"] = 2;


  MapType::iterator iterator = std::max_element(myMap.begin(), myMap.end(), ValueCompareFunctor());

  std::cout << iterator->first << " " << iterator->second << std::endl;
}
