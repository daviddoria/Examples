#include <set>

// Attempt 1
struct MyClass
{
  int const x;
  MyClass(int x): x(x) {}
  bool operator< (const MyClass &other) const;
};

bool MyClass::operator<(const MyClass &other) const
{
  if(this->x < other.x)
  {
    return true;
  }
  else if (other.x < this->x)
  {
    return false;
  }

}


int main()
{
  std::set<MyClass> container;
  container.insert(MyClass(3));
  return 0;
}

// Attempt 2
// /home/doriad/Test/Test.cxx:28:23: error: assignment of read-only member ‘MyClass::x’
// struct MyClass
// {
//   int const x;
//   MyClass(int x): x(x) {}
//   MyClass& operator= (const MyClass& other)
//   {
//     if (this != &other)
//     {
//       this->x = other.x;
//     }
//
//     return *this;
//   }
// };
//
// int main()
// {
//   std::vector<MyClass> vec;
//   vec.push_back(MyClass(3));
//   return 0;
// }

// Attempt 3
// /home/doriad/Test/Test.cxx:3:8: error: non-static const member ‘const int MyClass::x’, can’t use default assignment operator
// struct MyClass
// {
//   int const x;
//   MyClass(int x): x(x) {}
//   MyClass(const MyClass&other): x(other.x) { }
// };
//
// int main()
// {
//   std::vector<MyClass> vec;
//   vec.push_back(MyClass(3));
//   return 0;
// }

// Attempt 4
// This doesn't work
// struct MyClass
// {
//   int const x;
//   MyClass(int x): x(x) {}
//   MyClass(const MyClass&other): x(other.x) { }
// };
// 
// int main()
// {
//   std::vector<const MyClass> vec;
//   vec.push_back(MyClass(3));
//   return 0;
// }
