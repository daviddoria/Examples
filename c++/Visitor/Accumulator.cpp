#include <iostream>
#include <string>
#include <vector>

class IntElement;
class DoubleElement;

class Visitor
{
public:
  virtual void visit(IntElement *e) = 0;
  virtual void visit(DoubleElement *e) = 0;
};

class Element
{
public:
  virtual void accept(class Visitor &v) = 0;
};

class IntElement: public Element
{
public:
  IntElement(int i) : IntData(i){}
  /*virtual*/void accept(Visitor &v)
  {
    v.visit(this);
  }

  int IntData;
};

class DoubleElement: public Element
{
public:
  DoubleElement(double d) : DoubleData(d){}
  /*virtual*/void accept(Visitor &v)
  {
    v.visit(this);
  }

  double DoubleData;
};

class SumVisitor: public Visitor
{
public:
  SumVisitor() : Sum(0){}
  /*virtual*/void visit(IntElement *e)
  {
    Sum += e->IntData;
  }
  /*virtual*/void visit(DoubleElement *e)
  {
    Sum += e->DoubleData;
  }

  double Sum;
};

class AverageVisitor: public Visitor
{
public:
  AverageVisitor() : Counter(0) , Average(0){}
  /*virtual*/void visit(IntElement *e)
  {
    Counter++;
    Average = (Counter - 1) * Average + e->IntData;
    Average /= Counter;
  }
  /*virtual*/void visit(DoubleElement *e)
  {
    Counter++;
    Average = (Counter - 1) * Average + e->DoubleData;
    Average /= Counter;
  }
  double Average;
  int Counter;
};

int main()
{
  std::vector<Element*> elements;
  elements.push_back(new IntElement(0));
  elements.push_back(new IntElement(1));
  elements.push_back(new DoubleElement(2));
  elements.push_back(new DoubleElement(3));

  SumVisitor sumVisitor;
  AverageVisitor averageVisitor;
  for (int i = 0; i < elements.size(); i++)
    {
    elements[i]->accept(sumVisitor);
    elements[i]->accept(averageVisitor);
    }
  std::cout << "sum: " << sumVisitor.Sum << std::endl;
  std::cout << "average: " << averageVisitor.Average << std::endl;
}
