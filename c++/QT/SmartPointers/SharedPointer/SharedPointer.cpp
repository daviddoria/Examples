#include <QApplication>

#include <QLabel>

class MyClass
{
public:
  MyClass() : label(new QLabel){}
  QScopedPointer<QLabel> label;
  
};

int main( int argc, char **argv )
{
  QApplication app(argc, argv);

  QSharedPointer<QLabel> label1(new QLabel());

  QSharedPointer<QLabel> label2;
  //label2 = new QLabel; // Can't do this.
  label2 = QSharedPointer<QLabel>(new QLabel);

  MyClass myClass;
  return app.exec();
}
