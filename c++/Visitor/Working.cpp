#include <iostream>
#include <string>
#include <vector>

class Element
{
public:
  virtual void accept(class Visitor &v) = 0;
};

class ElementA: public Element
{
public:
  /*virtual*/void accept(Visitor &v);
  std::string elementAstring()
  {
    return "ElementA";
  }
};

class ElementB: public Element
{
public:
  /*virtual*/void accept(Visitor &v);
  std::string elementBstring()
  {
    return "ElementB";
  }
};

class Visitor
{
public:
  virtual void visit(ElementA *e) = 0;
  virtual void visit(ElementB *e) = 0;
};

/*virtual*/void ElementA::accept(Visitor &v)
{
  v.visit(this);
}

 /*virtual*/void ElementB::accept(Visitor &v)
{
  v.visit(this);
}


// 3. Create a "visitor" derived class for each "operation" to do on "elements"
class UpVisitor: public Visitor
{
    /*virtual*/void visit(ElementA *e)
  {
    std::cout << "do Up on " + e->elementAstring() << '\n';
  }
    /*virtual*/void visit(ElementB *e)
  {
    std::cout << "do Up on " + e->elementBstring() << '\n';
  }
};

class DownVisitor: public Visitor
{
  /*virtual*/void visit(ElementA *e)
  {
    std::cout << "do Down on " + e->elementAstring() << '\n';
  }
  /*virtual*/void visit(ElementB *e)
  {
    std::cout << "do Down on " + e->elementBstring() << '\n';
  }

};

int main()
{
  std::vector<Element*> elements;
  elements.push_back(new ElementA);
  elements.push_back(new ElementB);

  UpVisitor up;
  DownVisitor down;
  for (int i = 0; i < elements.size(); i++)
    {
    elements[i]->accept(up);
    elements[i]->accept(down);
    }
}
