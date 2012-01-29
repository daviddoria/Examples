#ifndef CALCULATORFORM_H
#define CALCULATORFORM_H

#include <QMainWindow>
#include <QStandardItemModel>

#include "ui_main.h"

class CheckBoxModel : public QStandardItemModel
{
  Qt::ItemFlags flags(const QModelIndex& index) const;

protected:

};

class MainWindow : public QMainWindow, public Ui::MainWindow
{
    Q_OBJECT

public:
    MainWindow(QMainWindow *parent = 0);
protected:
  CheckBoxModel* checkboxModel;
};

#endif
