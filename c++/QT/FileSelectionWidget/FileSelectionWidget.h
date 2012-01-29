#ifndef FileSelectionWidget_H
#define FileSelectionWidget_H

#include "ui_FileSelectionWidget.h"

#include <QMainWindow>

class QFileSystemModel;

class FileSelectionWidget : public QWidget, private Ui::FileSelectionWidget
{
  Q_OBJECT

public:
  FileSelectionWidget(QWidget *parent = 0);

public slots:
  void on_listView_doubleClicked(const QModelIndex & index);
  void on_listView_clicked(const QModelIndex & index);

protected:
  QFileSystemModel *model;
};

#endif
