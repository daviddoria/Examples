#include <highgui.h>
#include <cv.h>
#include <iostream>

using namespace std;
using namespace cv;

const Scalar GREEN = Scalar(0,255,0);

void getBinMask( const Mat& comMask, Mat& binMask )
{
    if( comMask.empty() || comMask.type()!=CV_8UC1 )
        CV_Error( CV_StsBadArg, "comMask is empty or has incorrect type (not CV_8UC1)" );
    if( binMask.empty() || binMask.rows!=comMask.rows || binMask.cols!=comMask.cols )
        binMask.create( comMask.size(), CV_8UC1 );
    binMask = comMask & 1;
}

class GCApplication
{
public:
    enum{ NOT_SET = 0, IN_PROCESS = 1, SET = 2 };
    static const int radius = 2;
    static const int thickness = -1;

    void reset();
    void setImageAndWinName( const Mat& _image, const string& _winName );
    void showImage() const;
    void mouseClick( int event, int x, int y, int flags, void* param );
    int nextIter();
//    int getIterCount() const { return iterCount; }
private:
    void setRectInMask();
    void setLblsInMask( int flags, Point p, bool isPr );

    const string* winName;
    const Mat* image;
    Mat mask;
    Mat bgdModel, fgdModel;

    uchar rectState, lblsState, prLblsState;
    bool isInitialized;

    Rect rect;
    //vector<Point> fgdPxls, bgdPxls, prFgdPxls, prBgdPxls;
    //int iterCount;
};

void GCApplication::setImageAndWinName( const Mat& _image, const string& _winName  )
{
    if( _image.empty() || _winName.empty() )
        return;
    image = &_image;
    winName = &_winName;
    mask.create( image->size(), CV_8UC1);
//    reset();
}

void GCApplication::showImage() const
{
    if( image->empty() || winName->empty() )
        return;

    Mat res;

    Mat binMask;
    if( !isInitialized )
        image->copyTo( res );
    else
    {
        getBinMask( mask, binMask );
        image->copyTo( res, binMask );
    }

    if( rectState == IN_PROCESS || rectState == SET )
        rectangle( res, Point( rect.x, rect.y ), Point(rect.x + rect.width, rect.y + rect.height ), GREEN, 2);

    imshow( *winName, res );
}

void GCApplication::setRectInMask()
{
    assert( !mask.empty() );
    mask.setTo( GC_BGD );
    rect.x = max(0, rect.x);
    rect.y = max(0, rect.y);
    rect.width = min(rect.width, image->cols-rect.x);
    rect.height = min(rect.height, image->rows-rect.y);
    (mask(rect)).setTo( Scalar(GC_PR_FGD) );
}

void GCApplication::mouseClick( int event, int x, int y, int flags, void* param )
{
    // TODO add bad args check
    switch( event )
    {
    case CV_EVENT_LBUTTONDOWN: // set rect or GC_BGD(GC_FGD) labels
        {
            if( rectState == NOT_SET)
            {
                rectState = IN_PROCESS;
                rect = Rect( x, y, 1, 1 );
            }
        }
        break;

    case CV_EVENT_LBUTTONUP:
        if( rectState == IN_PROCESS )
        {
            rect = Rect( Point(rect.x, rect.y), Point(x,y) );
            rectState = SET;
            setRectInMask();
            showImage();
        }
        break;
    case CV_EVENT_MOUSEMOVE:
        if( rectState == IN_PROCESS )
        {
            rect = Rect( Point(rect.x, rect.y), Point(x,y) );
            showImage();
        }
        break;
    }
}

int GCApplication::nextIter()
{

    if( this->isInitialized )
    {
      grabCut( *image, mask, rect, bgdModel, fgdModel, 1 );
    }
    else
    {
      grabCut( *image, mask, rect, bgdModel, fgdModel, 1, GC_INIT_WITH_RECT );
      this->isInitialized = true;
    }

 //grabCut( *image, mask, rect, bgdModel, fgdModel, 1, GC_INIT_WITH_RECT );
    return 1;
}

GCApplication gcapp;

void on_mouse( int event, int x, int y, int flags, void* param )
{
    gcapp.mouseClick( event, x, y, flags, param );
}

int main( int argc, char** argv )
{
    if( argc==1 )
        return 1;
    string filename = argv[1];
    if( filename.empty() )
        return 1;
    Mat image = imread( filename, 1 );
    if( image.empty() )
        return 1;

    const string winName = "image";
    cvNamedWindow( winName.c_str(), CV_WINDOW_AUTOSIZE );
    cvSetMouseCallback( winName.c_str(), on_mouse, 0 );

    gcapp.setImageAndWinName( image, winName );
    gcapp.showImage();

    for(;;)
    {
        int c = cvWaitKey(0);
        switch( (char) c )
        {
        case '\x1b':
            cout << "Exiting ..." << endl;
            goto exit_main;
        case 'n':
/*            int iterCount = gcapp.getIterCount();
            cout << "<" << iterCount << "... ";
            */
            int newIterCount = gcapp.nextIter();
            //if( newIterCount > iterCount )
            //{
                gcapp.showImage();
                //cout << iterCount << ">" << endl;
            //}
            break;
        }
    }

exit_main:
    cvDestroyWindow( winName.c_str() );
    return 0;
}

