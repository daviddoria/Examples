#include <QtGui>
#include <QTextCursor>
#include <QTextDocument>

#include "form.h"
#include <iostream>

MainWindow::MainWindow()
{
  setupUi(this);



  QTextDocument* myDocument = new QTextDocument(textEdit);
  textEdit->setDocument(myDocument);
  QTextCursor* myCursor = new QTextCursor(myDocument);

  QTextBlockFormat centerFormat;
  centerFormat.setAlignment(Qt::AlignHCenter);
  
  // Insert an image
  QTextImageFormat imageFormat;
  imageFormat.setName("headshot.jpg");
  myCursor->setBlockFormat(centerFormat);
  myCursor->insertImage(imageFormat, QTextFrameFormat::InFlow);
  myCursor->insertText("\n");
  
  // Insert normal text
  QTextBlockFormat format;
  format.setBackground(Qt::red);
  myCursor->setBlockFormat(format);
  myCursor->movePosition(QTextCursor::End);
  myCursor->insertText("the ");

  format.setBackground(Qt::green);
  myCursor->insertBlock(format);
  myCursor->insertText("fish ");

  format.setBackground(Qt::yellow);
  myCursor->insertBlock(format);
  myCursor->insertText("are ");

  format.setBackground(Qt::red);
  myCursor->insertBlock(format);
  myCursor->insertText("coming!");

  // Insert tables
  QTextTable* table = myCursor->insertTable (5, 2); // 5 rows, 2 cols
//   myCursor->insertText("cell0");
//   myCursor->movePosition(QTextCursor::NextCell);
//   myCursor->insertText("cell1");
  
  QTextTableCell cell = table->cellAt (3,0);
  QTextCursor cellCursor = cell.firstCursorPosition();
  cellCursor.insertText("cell 3,0");

  
  QTextTableFormat tableFormat;
  tableFormat.setAlignment(Qt::AlignHCenter);
  //myCursor->movePosition(QTextCursor::EndOfBlock);
  //myCursor->movePosition(QTextCursor::NextBlock);
  myCursor->movePosition(QTextCursor::End);
  QTextTable* table2 = myCursor->insertTable (3, 2, tableFormat);
}
