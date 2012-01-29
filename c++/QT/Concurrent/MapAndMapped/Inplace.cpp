#include <QFuture>

#include <QtConcurrentMap>

#include <iostream>
#include <vector>

void doubleValue(float &v)
{
  v *= 2.0f;
}

int main()
{
  std::vector<float> v;
  v.push_back(1.0);
  v.push_back(2.0);
  v.push_back(3.0);

  QtConcurrent::blockingMap(v, doubleValue);
  for(unsigned int i = 0; i < v.size(); i++)
    {
    std::cout << v[i] << std::endl;
    }

    
  return 0;
}
