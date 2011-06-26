#include <QVariant>

#include <iostream>

struct MyStruct
{
  int a;
};
 Q_DECLARE_METATYPE(MyStruct)
 
void StandardTypes();
void CustomTypes();

int main()
{
  StandardTypes();
  CustomTypes();

  return 0;
}

void StandardTypes()
{
  QVariant v;

  v.setValue(5);
  int i = v.toInt();         // i is now 5
  QString s = v.toString();   // s is now "5"
}

void CustomTypes()
{
  QVariant v;

  MyStruct c;
  v.setValue(c);

  MyStruct c2 = v.value<MyStruct>();
}
