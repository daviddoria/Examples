#include <iostream>
#include <vector>

void Output(std::vector<int>& v);
void RemoveElementWithId(std::vector<int>& v, const unsigned int id);
void RemoveAfter(std::vector<int>& v, const unsigned int id);

int main(int, char*[])
{
  // Create a vector
  std::vector<int> v(10);

  for(unsigned int i = 0; i < 10; i++)
  {
    v[i] = i;
  }

  Output(v);

  //RemoveElementWithId(v, 3);
  RemoveAfter(v, 5);
  Output(v);
  
  return 0;
}

void RemoveAfter(std::vector<int>& v, const unsigned int id)
{
  v.erase(v.begin() + id, v.end());
  //Output(v);
}

void RemoveElementWithId(std::vector<int>& v, const unsigned int id)
{
  //v.erase(v.begin() + id, v.begin() + id);
  v.erase(v.begin() + id);
}

void RemoveElementWithValue(std::vector<int>& v)
{
  // Remove an element with a particular value
  for(std::vector<int>::iterator it = v.begin(); it!= v.end(); it++)
  {
    if(*it == 4)
      {
      v.erase(it);
      }
  }  
}

void RemoveLastItem(std::vector<int>& v)
{

  // Remove the last item
  v.erase(v.end()-1);
  
  Output(v);
    
}

void Output(std::vector<int>& v)
{
  for(unsigned int i = 0; i < v.size(); i++)
    {
    std::cout << v[i] << " ";
    }
  std::cout << std::endl;
}
