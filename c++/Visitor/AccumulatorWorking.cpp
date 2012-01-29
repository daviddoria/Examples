#include <iostream>
#include <vector>

class Item
{
public:
  virtual int value() = 0;
};

class IntItem : public Item
{
public:
  IntItem(const int a) : data(a){}

  int value() {return data;}

private:
  int data;
};

class Visitor
{
public:
  virtual void Visit(Item &item) = 0;
};

class Accumulator : public Visitor
{
public:

  Accumulator() : sum(0) {}
  void Visit(Item &item)
  {
    sum += item.value();
  }

  int GetSum() { return sum;}

private:
  int sum;
};

class Collection
{
public:
  void Visit(Visitor &visitor)
  {
    for(unsigned int i = 0; i < data.size(); ++i)
      {
      visitor.Visit(data[i]);
      }
  }

  void AddItem(const IntItem& item)
  {
    data.push_back(item);
  }

private:
  std::vector<IntItem> data;
};

class MyClass
{
public:
  MyClass()
  {
    for(unsigned int i = 0; i < 4; ++i)
      {
      m_collection.AddItem(IntItem(i));
      }
  }

  int Sum()
  {
    Accumulator accumulator;
    m_collection.Visit(accumulator);
    return accumulator.GetSum();
  }

private:
  Collection m_collection;
};

int main(int argc, char *argv[])
{
  MyClass myClass;
  std::cout << myClass.Sum() << std::endl;

  return 0;
}
