#ifndef TEST_H
#define TEST_H

#include "ui_Test.h"

class Test : public QMainWindow
{
    Q_OBJECT

public:
	Test(QWidget *parent = 0){ui.setupUi(this);}

public slots:
    //void pushButton_SetLabelText();

private:
    Ui::MainWindow ui;
};

#endif
