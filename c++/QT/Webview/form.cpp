#include <QtGui>
#include <QUrl>

#include "form.h"
#include <iostream>

MainWindow::MainWindow()
{
  setupUi(this);
  QUrl url("http://www.google.com"); // Must have http:// or nothing appears.
  webView->load(url);
}
