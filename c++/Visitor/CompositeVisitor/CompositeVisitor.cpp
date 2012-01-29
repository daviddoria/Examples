#include <iostream>
#include <vector>

class Visitor
{
public:
  virtual void Visit(int &item) = 0;
};

class SumAccumulator : public Visitor
{
public:
  SumAccumulator() : Sum(0) {}
  void Visit(int &item)
  {
    this->Sum += item;
  }

  int Sum;
};

class AverageAccumulator : public Visitor
{
public:

  AverageAccumulator() : Average(0), Counter(0) {}
  void Visit(int &item)
  {
    Counter++;
    Average = (Counter - 1) * Average + item;
    Average /= Counter;
  }

  int Counter;
  float Average;
};

class CompositeVisitor : public Visitor
{
public:

  CompositeVisitor(){}
  void Visit(int &item)
  {
    for(unsigned int i = 0; i < this->Visitors.size(); ++i)
      {
      this->Visitors[i]->Visit(item);
      }
  }
  
  void AddVisitor(Visitor* visitor)
  {
    this->Visitors.push_back(visitor);
  }

private:
  std::vector<Visitor*> Visitors;
};

class IntCollection
{
public:
  void Visit(Visitor &visitor)
  {
    for(unsigned int i = 0; i < this->Items.size(); ++i)
      {
      visitor.Visit(this->Items[i]);
      }
  }

  void AddItem(const int& item)
  {
    this->Items.push_back(item);
  }

private:
  std::vector<int> Items;

};

int main(int argc, char *argv[])
{
  /////// Ints ////////
  IntCollection intCollection;
  for(unsigned int i = 0; i < 4; ++i)
    {
    intCollection.AddItem(i);
    }

  // Single visitor operations
//   SumAccumulator intSumAccumulator;
//   intCollection.Visit(intSumAccumulator);
//   std::cout << "sum: " << intSumAccumulator.Sum << std::endl;
// 
//   AverageAccumulator intAverageAccumulator;
//   intCollection.Visit(intAverageAccumulator);
//   std::cout << "average: " << intAverageAccumulator.Average << std::endl;

  SumAccumulator sumVisitor;
  AverageAccumulator averageVisitor;

  CompositeVisitor compositeVisitor;
  compositeVisitor.AddVisitor(&sumVisitor);
  compositeVisitor.AddVisitor(&averageVisitor);

  return 0;
}
