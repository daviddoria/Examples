#include <iostream>

#include <boost/timer.hpp>

// Policy version
template < typename operation_policy>
class DoOperationPolicy : public operation_policy
{
  using operation_policy::Operation;

public:
  float Run(const float a, const float b)
  {
    return Operation(a,b);
  }
};

class OperationPolicy_Add
{
protected:
  float Operation(const float a, const float b)
  {
    return a + b;
  }
};

// Polymorphic version
class DoOperation
{
public:
  virtual float Run(const float a, const float b)= 0;
};

class OperationAdd : public DoOperation
{
public:
  float Run(const float a, const float b)
  {
    return a + b;
  }
};

class OperationSubtract : public DoOperation
{
public:
  float Run(const float a, const float b)
  {
    return a - b;
  }
};

int main()
{
  // In debug mode, policy takes .13 seconds, while polymorphic takes only .05 seconds. (1e7 iterations)
  boost::timer timer;

  unsigned int numberOfIterations = 1e8;

  DoOperationPolicy<OperationPolicy_Add> policy_operation;
  float a;
  for(unsigned int i = 0; i < numberOfIterations; ++i)
    {
    a += policy_operation.Run(1,2);
    }
  std::cout << timer.elapsed() << " seconds." << std::endl;
  timer.restart();

  DoOperation* polymorphic_operation = new OperationAdd;
  for(unsigned int i = 0; i < numberOfIterations; ++i)
    {
    a += polymorphic_operation->Run(1,2);
    }
  std::cout << timer.elapsed() << " seconds." << std::endl;

}
