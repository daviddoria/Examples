#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include "ClickableScene.h"

class Form : public QWidget, private Ui::Form
{
    Q_OBJECT

public:
    Form(QWidget *parent = 0);
    
public slots:
  //void slot_clicked( QGraphicsSceneMouseEvent * mouseEvent );
  
protected:
  ClickableScene* Scene;
};

#endif
