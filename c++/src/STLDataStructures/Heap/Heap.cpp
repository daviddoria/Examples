#include <iostream>
#include <algorithm>
#include <functional>
#include <vector>

using namespace std;

class SimpleClass
{
  public:
  SimpleClass(){}
  SimpleClass(int value, std::string name) {this->Value = value; this->Name = name;}
  int Value;
  std::string Name;

  bool operator< (const SimpleClass &b)
  {
      if(this->Value < b.Value)
      {
        return true;
      }
      else
      {
        return false;
      }
  }
};
/*
  bool operator< (const SimpleClass &a, const SimpleClass &b)
  {
      if(a.Value < b.Value)
      {
        return true;
      }
      else
      {
        return false;
      }
  }
  */

void TestStandard();
void TestCustom();

void Print(std::vector<int> v);
void Print(std::vector<SimpleClass> v);

int main()
{
  //TestStandard();
  TestCustom();

  return 0;
}

void TestStandard()
{
  // Define a template class vector of int
  vector<int> Numbers(8);

  // Initialize vector Numbers
  Numbers[0] = 4 ;
  Numbers[1] = 10;
  Numbers[2] = 70 ;
  Numbers[3] = 10 ;
  Numbers[4] = 30 ;
  Numbers[5] = 69 ;
  Numbers[6] = 96 ;
  Numbers[7] = 100;

  cout << "Initial numbers:" << std::endl;
  Print(Numbers);

  // convert Numbers into a heap
  make_heap(Numbers.begin(), Numbers.end()) ;

  cout << "After calling make_heap" << endl ;
  Print(Numbers);

  // sort the heapified sequence Numbers
  sort_heap(Numbers.begin(), Numbers.end()) ;

  cout << "After calling sort_heap" << endl ;
  Print(Numbers);

  //insert an element in the heap
  Numbers.push_back(7) ;
  push_heap(Numbers.begin(), Numbers.end()) ;

  // you need to call make_heap to re-assert the
  // heap property
  make_heap(Numbers.begin(), Numbers.end()) ;

  cout << "After calling push_heap and make_heap" << endl ;
  Print(Numbers);

  // remove the root element from the heap Numbers
  pop_heap(Numbers.begin(), Numbers.end()) ;

  cout << "After calling pop_heap" << endl ;
  Print(Numbers);
}


void TestCustom()
{
  std::vector<SimpleClass> Numbers(8);

  // Initialize vector Numbers
  Numbers[0] = SimpleClass(4, "four") ;
  Numbers[1] = SimpleClass(10, "ten");
  Numbers[2] = SimpleClass(70, "seventy") ;
  Numbers[3] = SimpleClass(10, "ten") ;
  Numbers[4] = SimpleClass(30, "thirty") ;
  Numbers[5] = SimpleClass(69, "sixtynine") ;
  Numbers[6] = SimpleClass(96, "ninetysix") ;
  Numbers[7] = SimpleClass(100, "onehundred");

  std::cout << "Initial numbers:" << std::endl;
  Print(Numbers);

  // convert Numbers into a heap
  make_heap(Numbers.begin(), Numbers.end()) ;

  cout << "After calling make_heap" << endl ;
  Print(Numbers);

  // sort the heapified sequence Numbers
  sort_heap(Numbers.begin(), Numbers.end()) ;

  cout << "After calling sort_heap" << endl ;
  Print(Numbers);

  //insert an element in the heap
  Numbers.push_back(SimpleClass(7,"seven")) ;
  push_heap(Numbers.begin(), Numbers.end()) ;

  // you need to call make_heap to re-assert the
  // heap property
  make_heap(Numbers.begin(), Numbers.end()) ;

  cout << "After calling push_heap and make_heap" << endl ;
  Print(Numbers);

  // remove the root element from the heap Numbers
  pop_heap(Numbers.begin(), Numbers.end()) ;
  cout << "After calling pop_heap" << endl ;
  Print(Numbers);

}

void Print(std::vector<int> v)
{
  cout << "{ " ;
  for(std::vector<int>::iterator it = v.begin(); it != v.end(); it++)
      cout << *it << " " ;
  cout << " }" << std::endl << std::endl;
}

void Print(std::vector<SimpleClass> v)
{
  cout << "{ " ;
  for(std::vector<SimpleClass>::iterator it = v.begin(); it != v.end(); it++)
      cout << (*it).Value << " " ;
  cout << " }" << std::endl << std::endl;
}