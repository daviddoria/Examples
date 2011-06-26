
 #include <QtGui>

 #include "calculatorform.h"

 CalculatorForm::CalculatorForm(QWidget *parent)
     : QWidget(parent)
 {
     ui.setupUi(this);
 }

 void CalculatorForm::on_inputSpinBox1_valueChanged(int value)
 {
     ui.outputWidget->setText(QString::number(value + ui.inputSpinBox2->value()));
 }

 void CalculatorForm::on_inputSpinBox2_valueChanged(int value)
 {
     ui.outputWidget->setText(QString::number(value + ui.inputSpinBox1->value()));
 }
