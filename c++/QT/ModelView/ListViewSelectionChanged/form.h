#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include <QMainWindow>

class QStringListModel;

class Form : public QMainWindow, private Ui::MainWindow
{
  Q_OBJECT

public:
  Form(QWidget *parent = 0);

public slots:
  void on_listView_clicked(const QModelIndex & index);

protected:
  QStringListModel *model;
};

#endif
