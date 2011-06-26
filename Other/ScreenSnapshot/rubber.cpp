// g++ rubber.cpp -o ss -lQtGui

#include <QtGui/QWidget> 
#include <QtGui/QDesktopWidget>
#include <QtGui/QApplication>
#include <QtGui/QPixmap>
#include <QtGui/QRubberBand>
#include <QtGui/QMouseEvent>
#include <QtGui/QPushButton>
#include <QtGui/QFont>
#include <QtGui/QPainter>

#include <iostream>

using namespace std;

class MyWidget : public QWidget
{
public:
	MyWidget(QWidget *parent = 0);
	QRubberBand outline();

protected:
	void mousePressEvent(QMouseEvent *event);

};

MyWidget::MyWidget(QWidget *parent) : QWidget(parent)
{

}

int main(int argc, char *argv[])
{
	QApplication MyScreenshot(argc,argv);
	QPoint TopLeft(100,100);
	QPoint BottomRight(200,200);
	QRect SelectionRectangle(TopLeft, BottomRight);

	MyWidget A();
	return MyScreenshot.exec();
}

void MyWidget::mousePressEvent(QMouseEvent *event)
{

	QPoint TopLeft(100, 200);
	QPoint BottomRight(200,200);
	QRect SelectionRectangle(TopLeft, BottomRight);
	outline.setGeometry(SelectionRectangle);
	outline.show();
}

