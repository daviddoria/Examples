
 #ifndef CALCULATORFORM_H
 #define CALCULATORFORM_H

 #include "ui_calculatorform.h"

 class CalculatorForm : public QWidget
 {
     Q_OBJECT

 public:
     CalculatorForm(QWidget *parent = 0);

 private slots:
     void on_inputSpinBox1_valueChanged(int value);
     void on_inputSpinBox2_valueChanged(int value);

 private:
     Ui::CalculatorForm ui;
 };

 #endif
