#ifndef MainWindow_H
#define MainWindow_H

#include <QMainWindow>
#include <QStandardItemModel>

#include "ui_main.h"

class MainWindow : public QMainWindow, public Ui::MainWindow
{
Q_OBJECT

public:
  MainWindow(QMainWindow *parent = 0);

public slots:

  //void slot_changed();
  void slot_changed(const QModelIndex&, const QModelIndex&);
  
protected:
  QStandardItemModel* Model;
  QStandardItem* Item1;
  QStandardItem* Item2;

  std::vector<QStandardItem*> Items;
};

#endif
