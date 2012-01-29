#include <iostream>
#include <vector>

class IntItem
{
public:
  IntItem(const int a) : IntData(a){}

  int IntData;
};

class DoubleItem
{
public:
  DoubleItem(const double a) : DoubleData(a){}

  double DoubleData;
};

class Visitor
{
public:
  virtual void Visit(IntItem &item) = 0;
  virtual void Visit(DoubleItem &item) = 0;
};


class SumAccumulator : public Visitor
{
public:

  SumAccumulator() : Sum(0) {}
  void Visit(IntItem &item)
  {
    Sum += item.IntData;
  }

  void Visit(DoubleItem &item)
  {
    Sum += item.DoubleData;
  }

  double Sum;
};

class AverageAccumulator : public Visitor
{
public:

  AverageAccumulator() : Average(0), Counter(0) {}
  void Visit(IntItem &item)
  {
    Counter++;
    Average = (Counter - 1) * Average + item.IntData;
    Average /= Counter;
  }

  void Visit(DoubleItem &item)
  {
    Counter++;
    Average = (Counter - 1) * Average + item.DoubleData;
    Average /= Counter;
  }

  int Counter;
  double Average;
};

class IntCollection
{
public:
  void Visit(Visitor &visitor)
  {
    for(unsigned int i = 0; i < IntItems.size(); ++i)
      {
      visitor.Visit(IntItems[i]);
      }
  }

  void AddIntItem(const IntItem& item)
  {
    IntItems.push_back(item);
  }

private:
  std::vector<IntItem> IntItems;

};

class DoubleCollection
{
public:
  void Visit(Visitor &visitor)
  {
    for(unsigned int i = 0; i < DoubleItems.size(); ++i)
      {
      visitor.Visit(DoubleItems[i]);
      }
  }

  void AddDoubleItem(const DoubleItem& item)
  {
    DoubleItems.push_back(item);
  }

private:
  std::vector<DoubleItem> DoubleItems;
};

int main(int argc, char *argv[])
{
  /////// Ints ////////
  IntCollection intCollection;
  for(unsigned int i = 0; i < 4; ++i)
    {
    intCollection.AddIntItem(IntItem(i));
    }

  SumAccumulator intSumAccumulator;
  intCollection.Visit(intSumAccumulator);
  std::cout << "int sum: " << intSumAccumulator.Sum << std::endl;

  AverageAccumulator intAverageAccumulator;
  intCollection.Visit(intAverageAccumulator);
  std::cout << "int average: " << intAverageAccumulator.Average << std::endl;

  /////// Doubles ////////
  DoubleCollection doubleCollection;
  for(unsigned int i = 0; i < 4; ++i)
    {
    doubleCollection.AddDoubleItem(DoubleItem(static_cast<double>(i) + .1));
    }
  SumAccumulator doubleSumAccumulator;
  doubleCollection.Visit(doubleSumAccumulator);
  std::cout << "double sum: " << doubleSumAccumulator.Sum << std::endl;

  AverageAccumulator doubleAverageAccumulator;
  doubleCollection.Visit(doubleAverageAccumulator);
  std::cout << "double average: " << doubleAverageAccumulator.Average << std::endl;

  return 0;
}
