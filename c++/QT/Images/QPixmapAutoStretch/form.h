#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include <iostream>
#include <QGraphicsScene>
#include <QGraphicsView>

class CustomGraphicsView: public QGraphicsView
{
  Q_OBJECT
  
  void resizeEvent ( QResizeEvent * event )
  {
    emit resized();
  }

signals:
  void resized();
};

class Form : public QWidget, private Ui::Form
{
    Q_OBJECT

public:
  Form(QWidget *parent = 0);

public slots:
  void SizeImage();
  
protected:

  QGraphicsScene* Scene;
  CustomGraphicsView* View;
  QPixmap Pixmap;
  QGraphicsPixmapItem* PixmapItem;

};

#endif
