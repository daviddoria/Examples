#ifndef FORM_H
#define FORM_H

#include <QWidget>

class QVBoxLayout;
class QCheckBox;

class MyForm : public QWidget
{
Q_OBJECT

public:
  MyForm(QWidget *parent = 0);

public slots:
  void Add();
  void Remove();

protected:
  QVBoxLayout* Layout;
  std::vector<QCheckBox*> CheckBoxes;
};

#endif
