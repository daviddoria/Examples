#include <QFuture>

#include <QtConcurrentMap>

#include <iostream>
#include <vector>

float doubleValue(const float v)
{
  return v * 2.0f;
}


void Blocking(const std::vector<float>& v);
void NonBlocking(const std::vector<float>& v);

int main()
{
  std::vector<float> v;
  v.push_back(1.0);
  v.push_back(2.0);
  v.push_back(3.0);
  
  //Blocking(v);
  //NonBlocking(v);
  
  return 0;
}

void Blocking(const std::vector<float>& v)
{
  std::vector<float> doubledValues = QtConcurrent::blockingMapped<std::vector<float> >(v, doubleValue);
  for(unsigned int i = 0; i < doubledValues.size(); i++)
    {
    std::cout << doubledValues[i] << std::endl;
    }
}

void NonBlocking(const std::vector<float>& v)
{
  QFuture<float> doubledValues = QtConcurrent::mapped(v, doubleValue);
  
  while(!doubledValues.isFinished())
  {
    // wait
  }
  
  for(unsigned int i = 0; i < doubledValues.resultCount(); i++)
    {
    std::cout << doubledValues.resultAt(i) << std::endl;
    }
  
}
