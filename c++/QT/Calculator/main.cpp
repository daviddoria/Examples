
 #include <QApplication>

 #include "calculatorform.h"

 int main(int argc, char *argv[])
 {
     QApplication app(argc, argv);
     CalculatorForm calculator;
     calculator.show();
     return app.exec();
 }
