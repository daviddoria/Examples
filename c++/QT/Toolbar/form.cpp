#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QMainWindow(parent)
{
  setupUi(this);

  QIcon saveIcon = QIcon::fromTheme("document-save");
  actionSave->setIcon(saveIcon);
  this->toolBar->addAction(actionSave);
}
