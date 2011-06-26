// g++ main.cpp screenshot.cpp -I/usr/include/QtGui -lQtGui
 
 #include <QApplication>

 #include "screenshot.h"

 int main(int argc, char *argv[])
 {
     QApplication app(argc, argv);
     Screenshot screenshot;
     screenshot.show();
	screenshot.shootScreen();
	screenshot.saveScreenshot("test.png");
     //return app.exec();
 }
