#include <iostream>
#include <map>

typedef std::map<int, float> MapType;

MapType create_map()
{
  MapType m;
  m[1] = 2;
  m[3] = 4;
  m[5] = 6;
  return m;
}

static std::map<int, float> MyMap = create_map();

// Option 2:
// bool create_map(MapType &m)
// {
//   m[1] = 2;
//   m[3] = 4;
//   m[5] = 6;
//   return true;
// }
// static std::map<int, float> MyMap;
// static bool dummy = create_map(MyMap);


int main()
{
  std::cout << MyMap[1];
}