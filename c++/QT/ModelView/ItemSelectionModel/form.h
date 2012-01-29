#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include <QMainWindow>

class QStringList;
class QStringListModel;
class QItemSelectionModel;

class Form : public QMainWindow, private Ui::MainWindow
{
  Q_OBJECT

public:
  Form(QWidget *parent = 0);

public slots:
  void slot_selectionChanged (const QItemSelection  &selected, const QItemSelection  &deselected );

protected:
  QStringListModel* Model;
  QItemSelectionModel* SelectionModel;
  QStringList* StringList;
};

#endif
