#include <QtConcurrentMap>
#include <QVector>

#include <iostream>

class Object{};

float Function(Object& object) {}

int main()
{
  QVector<Object> objects;
  Object object;
  objects.push_back(object);

  QVector<float> result = QtConcurrent::blockingMapped<QVector<float> >(objects.begin(), objects.end(), &Function);

  return 0;
}
