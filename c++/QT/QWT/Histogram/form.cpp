#include "form.h"

#include <qwt_plot.h>
#include <qwt_plot_histogram.h>
//#include <qwt_plot_curve.h>
#include <qwt_series_data.h>


MyForm::MyForm(QWidget *parent) : QWidget(parent)
{
  setupUi(this);

  QwtPlotHistogram* histogram = new QwtPlotHistogram( "TestHistogram");
  
  QwtPlot* myPlot = new QwtPlot(this);

  unsigned int numValues = 4;
  QVector<QwtIntervalSample> samples(numValues);

  for ( uint i = 0; i < numValues; i++ )
    {
    QwtInterval interval( double( i ), i + 1.0 );
    interval.setBorderFlags( QwtInterval::ExcludeMaximum );
    samples[i] = QwtIntervalSample( i, interval );
    }

  QwtIntervalSeriesData* myData = new QwtIntervalSeriesData(samples);
    
  histogram->setData(myData);
  
  histogram->attach(myPlot);
  
  myPlot->replot();
}
