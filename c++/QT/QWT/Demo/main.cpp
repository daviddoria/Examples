#include <qwt_scale_map.h>
#include <qwt_plot_curve.h>
#include <qwt_symbol.h>
#include <qwt_math.h>
#include <qcolor.h>
#include <qpainter.h>
#include <qapplication.h>
#include <qframe.h>

//
//   Array Sizes
//
const int Size = 27;
const int CurvCnt = 1;

//
//   Arrays holding the values
//
double xval[Size];
double yval[Size];
QwtScaleMap xMap;
QwtScaleMap yMap;

class MainWin : public QFrame
{
public:
    MainWin();

protected:
    virtual void paintEvent( QPaintEvent * );
    void drawContents( QPainter *p );

private:
    void shiftDown( QRect &rect, int offset ) const;

    QwtPlotCurve curve;
};

MainWin::MainWin()
{
    int i;

    xMap.setScaleInterval( -0.5, 10.5 );
    yMap.setScaleInterval( -1.1, 1.1 );

    //
    //  Frame style
    //
    setFrameStyle( QFrame::Box | QFrame::Raised );
    setLineWidth( 2 );
    setMidLineWidth( 3 );

    //
    // Calculate values
    //
    for( i = 0; i < Size; i++ )
    {
        xval[i] = double( i ) * 10.0 / double( Size - 1 );
        yval[i] = qSin( xval[i] ) * qCos( 2.0 * xval[i] );
    }

    curve.setRawSamples( xval, yval, Size );
}

void MainWin::shiftDown( QRect &rect, int offset ) const
{
    rect.translate( 0, offset );
}

void MainWin::paintEvent( QPaintEvent *event )
{
    QFrame::paintEvent( event );

    QPainter painter( this );
    painter.setClipRect( contentsRect() );
    drawContents( &painter );
}

void MainWin::drawContents( QPainter *painter )
{
    int deltay, i;

    QRect r = contentsRect();

    deltay = r.height() / CurvCnt - 1;

    r.setHeight( deltay );


    xMap.setPaintInterval( r.left(), r.right() );
    yMap.setPaintInterval( r.top(), r.bottom() );

    painter->setRenderHint( QPainter::Antialiasing,
	curve.testRenderHint( QwtPlotItem::RenderAntialiased ) );
    curve.draw( painter, xMap, yMap, r );

}

int main ( int argc, char **argv )
{
    QApplication a( argc, argv );

    MainWin w;

    w.resize( 300, 600 );
    w.show();

    return a.exec();
}
