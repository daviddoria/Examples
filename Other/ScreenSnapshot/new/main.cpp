//compile and link with
// g++ main.cpp -o ss -I/usr/include/QtGui -lQtGui

#include <QWidget> 
#include <QDesktopWidget>
#include <QApplication>
#include <QPixmap>
#include <QRubberBand>
#include <iostream>

using namespace std;

void shootScreen(char* fileName);

int main(int argc, char *argv[])
{
	QApplication MyScreenshot(argc,argv);
	shootScreen("test.png");


	QPoint TopLeft(100,100);
	QPoint BottomRight(200,200);
	QRect SelectionRectangle(TopLeft, BottomRight);

	QRubberBand outline (QRubberBand::Rectangle);
	outline.setGeometry(SelectionRectangle);
	outline.show();
	
	//pause
	int a;
	cin >> a;

	return 0;
}

void shootScreen(char* fileName)
{
	QPixmap originalPixmap;
	originalPixmap = QPixmap::grabWindow(QApplication::desktop()->winId(), 100, 500, 200 , 50);//x, y, width, height
	originalPixmap.save(fileName);
}

