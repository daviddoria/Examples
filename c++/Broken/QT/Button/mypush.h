
 #ifndef CALCULATORFORM_H
 #define CALCULATORFORM_H

#include <QMainWindow>

 #include "ui_main.h"

 class MainWindow : public QMainWindow
 {
     Q_OBJECT

 public:
     MainWindow(QMainWindow *parent = 0);

 private slots:
     
 private:
     Ui::MainWindow ui;
 };

 #endif
