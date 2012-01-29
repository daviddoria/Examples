#include <iostream>

class UpdateRule;

class A
{
public:
  A(){this->Data = 4; this->MyConstant = 3;}
  float Data;

  float MyConstant;

public:
  UpdateRule* updateRule;
  void Update();
};

class UpdateRule
{
public:
  virtual void Update(A& a) = 0;
};

class UpdateRule1 : public UpdateRule
{
public:
  void Update(A& a){a.Data += a.MyConstant;}
};

class UpdateRule2 : public UpdateRule
{
public:
  void Update(A& a){a.Data += 2*a.MyConstant;}
};

int main(int argc, char* argv[])
{
  A myObject;
  myObject.updateRule = new UpdateRule2;
  myObject.Update();
  std::cout << myObject.Data << std::endl;
  return 0;
}

void A::Update()
{
  this->updateRule->Update(*this);
}