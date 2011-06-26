#include "form.h"

#include <KFileWidget>
#include <KUrl>

#include <iostream>

class CustomFileWidget : public KFileWidget
{
  Q_OBJECT
public:
  CustomFileWidget(const KUrl &url, QWidget* parent) : KFileWidget(url, parent){}

  void mouseDoubleClickEvent ( QMouseEvent * event )
  {
    emit signal_fileDoubleClicked();
  }

signals:
  void signal_fileDoubleClicked();
};

MyForm::MyForm(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  CustomFileWidget* fileWidget = new CustomFileWidget( KUrl("/home"), this );
  fileWidget->show();

  connect( fileWidget, SIGNAL( signal_fileDoubleClicked() ), this, SLOT(slot_fileDoubleClicked()) );
  //connect( fileWidget, SIGNAL( dirOperator() ), this, SLOT(slot_fileDoubleClicked()) );
}

void MyForm::slot_fileDoubleClicked()
{
  std::cout << "File double clicked" << std::endl;
}

#include "form.moc"
#include "moc_form.moc"