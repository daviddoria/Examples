#include <QtGui>
#include <QTextDocument>

#include "form.h"
#include <iostream>

// TextBlock is read only?

MainWindow::MainWindow()
{
  setupUi(this);

  QTextDocument* document = new QTextDocument;


  // QTextBlock - standard text

  textEdit->setDocument(document);
  // QTextTable - table

  
  // QTextImageFormat - Image

//        if (fragment.isValid()) {
//          QTextImageFormat newImageFormat = fragment.charFormat().toImageFormat();
// 
//          if (newImageFormat.isValid()) {
//              newImageFormat.setName(":/images/newimage.png");
//              QTextCursor helper = cursor;
// 
//              helper.setPosition(fragment.position());
//              helper.setPosition(fragment.position() + fragment.length(),
//                                  QTextCursor::KeepAnchor);
//              helper.setCharFormat(newImageFormat);
//          }
//      }
     
}
