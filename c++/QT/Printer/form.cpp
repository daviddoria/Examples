#include <QtGui>
#include <QGraphicsTextItem>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  this->setupUi(this);

  Scene = new QGraphicsScene();
  this->graphicsView->setScene(Scene);

  QPixmap blackPixmap(20, 20);
  QColor black(0,0,0);
  blackPixmap.fill(black);

  QPixmap redPixmap(20, 20);
  QColor red(255,0,0);
  redPixmap.fill(red);

  Scene->addPixmap(redPixmap);
  QGraphicsPixmapItem* item = Scene->addPixmap(blackPixmap);
  item->setPos(20,20);

  QGraphicsTextItem* simpleTextItem = Scene->addText("test");
  simpleTextItem->setPos(0, simpleTextItem->boundingRect().height());

}

void Form::on_btnSave_clicked()
{
  QString fileName = QFileDialog::getSaveFileName(this, "Save Scene", "", "PDF (*.pdf)");

  QPrinter printer;
  printer.setOutputFormat(QPrinter::PdfFormat);
  //printer.setPaperSize(QPrinter::Letter);
  printer.setOutputFileName(fileName);
  QPainter painter(&printer);
  Scene->render(&painter);

}
