#include <iostream>
#include <vector>

class Person
{
public:
  int Value;
};

int main()
{
  std::vector<Person*> people;
  Person* A = new Person;
  people.push_back(A);
  people.push_back(A);
  people.push_back(A);

  std::cout << A->Value;

  std::cout << people[0]->Value;

  std::cout << (*(people[0])).Value;

  return 0;
}