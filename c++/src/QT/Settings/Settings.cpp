#include <QtCore>

#include <iostream>
using namespace std;

int main(int argc, char *argv[])
{

  QCoreApplication app(argc, argv);
  
  QSettings settings("OpenSourcePress", "ConfigTest"); // (manufacturer, product)
  QString hello = "Hello, world!";
  // store a value
  settings.setValue("Greeting", hello); // creates ~/.config/OpenSourcePress/ConfigTest.conf
    
  // reset variable
  hello = "";
  // read value and assign to variable
  hello = settings.value("Greeting").toString();
  qDebug() << hello; // prints "Hello, world!"

  return 0;
}
