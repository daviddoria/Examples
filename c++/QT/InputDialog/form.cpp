#include "form.h"

#include <QMessageBox>
#include <QInputDialog>
#include <QDir>

MyForm::MyForm(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  connect( this->btnClickMe, SIGNAL( clicked() ), this, SLOT(btnClickMe_clicked()) );
}

void MyForm::btnClickMe_clicked()
{
  bool ok;
  QString text = QInputDialog::getText(this, tr("QInputDialog::getText()"),
                                        tr("User name:"), QLineEdit::Normal,
                                        QDir::home().dirName(), &ok);
  QMessageBox msgBox;
  if (ok && !text.isEmpty())
  {
    msgBox.setText("You said: " + text);
  }
  else
  {
    msgBox.setText("Error");
  }
   
  msgBox.exec();
}
