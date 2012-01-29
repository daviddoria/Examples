#include <QtCore>

#include <iostream>

void LocalUsage();
void GlobalUsage();

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);

  LocalUsage();
  GlobalUsage();

  return 0;
}

void LocalUsage()
{
  QSettings settings("MyCompany", "MyProgram");
  QString hello = "Hello, world!";
  // store a value
  settings.setValue("Greeting", hello); // creates ~/.config/OpenSourcePress/ConfigTest.conf

  // reset variable
  hello = "";
  // read value and assign to variable
  hello = settings.value("Greeting").toString();
  std::cout << hello.toStdString() << std::endl; // prints "Hello, world!"

}

void GlobalUsage()
{
  QCoreApplication::setOrganizationName("MyCompany");
  //QCoreApplication::setOrganizationDomain("mysoft.com");
  QCoreApplication::setApplicationName("MyProgram");
  
  QSettings settings;
  QString hello = "Hello, world!";
  // store a value
  settings.setValue("Greeting", hello); // creates ~/.config/OpenSourcePress/ConfigTest.conf

  // reset variable
  hello = "";
  // read value and assign to variable
  hello = settings.value("Greeting").toString();
  std::cout << hello.toStdString() << std::endl; // prints "Hello, world!"
}
