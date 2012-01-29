#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include <QMainWindow>

class QFileSystemModel;

class Form : public QMainWindow, private Ui::MainWindow
{
  Q_OBJECT

public:
  Form(QWidget *parent = 0);

public slots:
  void on_listView_doubleClicked(const QModelIndex & index);
  void on_listView_clicked(const QModelIndex & index);

protected:
  QFileSystemModel *model;
};

#endif
