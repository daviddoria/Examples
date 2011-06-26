/*
 * In QtDesigner, right click on the form and "Add Toolbar" to create a toolbar in which to programmatically add icons
 */

#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QMainWindow(parent)
{
  setupUi(this);

  QIcon openIcon = QIcon::fromTheme("document-open");
  QAction* openAction = new QAction(openIcon, "&Open...", this);
  toolBar->addAction(openAction);

  QIcon saveIcon = QIcon::fromTheme("document-save");
  QAction* saveAction = new QAction(saveIcon, "&Save...", this);
  toolBar->addAction(saveAction);

  // Can also use the following if the action is already created (i.e. by adding a QtDesigner file menu item)
  //openAction->setIcon(openIcon);
}
