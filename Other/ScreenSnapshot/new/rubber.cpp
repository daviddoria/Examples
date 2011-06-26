// g++ rubber.cpp -o ss -lQtGui

#include <QtGui/QWidget> 
#include <QtGui/QDesktopWidget>
#include <QtGui/QApplication>
#include <QtGui/QPixmap>
#include <QtGui/QRubberBand>
#include <QtGui/QMouseEvent>

#include <iostream>

using namespace std;

void mouseMoveEvent( QMouseEvent * e);
void mousePressEvent(QMouseEvent *event);

int main(int argc, char *argv[])
{
	QApplication MyScreenshot(argc,argv);

	QPoint TopLeft(100,100);
	QPoint BottomRight(200,200);
	QRect SelectionRectangle(TopLeft, BottomRight);

	QRubberBand outline (QRubberBand::Rectangle);
	outline.setGeometry(SelectionRectangle);
	outline.show();
	
	return MyScreenshot.exec();
}

void mouseMoveEvent( QMouseEvent * e)
{
	cout << "Mouse moved!" << endl;
}

void mousePressEvent(QMouseEvent *event)
{
	if (event->button() == Qt::LeftButton) 
	{
		cout << "Left click!" << endl;
	}
 }
