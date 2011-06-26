#include "form.h"

#include <KFileWidget>
#include <KUrl>

#include <iostream>

MyForm::MyForm(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  KFileWidget* fileWidget = new KFileWidget( KUrl("/home"), this );
  //fileWidget->setMode(KFile::Files); // if this is set, neither fileSelected or fileHighlighted are called
  fileWidget->show();

  connect( fileWidget, SIGNAL( fileSelected(const QString&) ), this, SLOT(fileSelected()) );
  connect( fileWidget, SIGNAL( fileHighlighted(const QString&) ), this, SLOT(fileHighlighted()) );
}

void MyForm::fileSelected()
{
  std::cout << "File selected." << std::endl;
}

void MyForm::fileHighlighted()
{
  std::cout << "File highlighted." << std::endl;
}
