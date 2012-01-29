#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

class Form : public QWidget, private Ui::Form
{
    Q_OBJECT
public slots:
  void on_pushButton_clicked();
  //void on_graphicsView_sceneRectChanged( const QRectF & rect );
  void slot_Scene_sceneRectChanged( const QRectF & rect );
  
public:
  Form(QWidget *parent = 0);

protected:
  void SizeImage();

  QGraphicsScene* Scene;
  QPixmap Pixmap;
};

#endif
