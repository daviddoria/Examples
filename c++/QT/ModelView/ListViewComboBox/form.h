#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include <QMainWindow>

#include "ComboBoxDelegate.h"

class QStringListModel;

class Form : public QMainWindow, private Ui::MainWindow
{
  Q_OBJECT

public:
  Form(QWidget *parent = 0);

public slots:
  void on_btnAdd_clicked();
  void slot_modelChanged(const QModelIndex  &topLeft, const QModelIndex  &bottomRight);

protected:
  QStringListModel* Model;
  ComboBoxDelegate Delegate;
};

#endif
