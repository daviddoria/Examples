/*
 *
 * The errors generated here are fixed by splitting the class definition and implementation into actual .h and .cpp files and then adding:

QT4_WRAP_CPP(MOCSrcs MyWidget.h)
ADD_EXECUTABLE(Signal Signal.cpp MyWidget.cpp ${MOCSrcs})

in the CMakeLists.txt
*/

#include <QApplication>

#include <iostream>

class MyWidget : public QObject
{
Q_OBJECT

public:
  //MyWidget(QWidget *parent = 0);
  MyWidget();

  void EmitSignal();

public slots:
  void myslot();

signals:
  void mysignal();



};

void MyWidget::mysignal()
{
  std::cout << "Signal emitted!" << std::endl;
}

void MyWidget::myslot()
{
  std::cout << "Slot called!" << std::endl;
}

void MyWidget::EmitSignal()
{
  emit mysignal();
}

MyWidget::MyWidget()
{
  connect( this, SIGNAL( mysignal() ), this, SLOT(myslot()) );
}

int main(int argc, char *argv[])
{
  QApplication a(argc,argv);

  MyWidget form;
  form.EmitSignal();

  return a.exec();
}

#include "mayn.moc"