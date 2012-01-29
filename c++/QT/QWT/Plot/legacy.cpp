#include "form.h"

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_series_data.h>


MyForm::MyForm(QWidget *parent) : QWidget(parent)
{
  setupUi(this);

  QwtPlot *myPlot = new QwtPlot(this);
  
  QwtPlotCurve *curve1 = new QwtPlotCurve("Curve 1");

  
  QwtPointSeriesData myData;
  QVector<QPointF> samples;
  samples.push_back(QPointF(1.0,1.0));
  samples.push_back(QPointF(2.0,2.0));
  samples.push_back(QPointF(3.0,3.0));
  samples.push_back(QPointF(4.0,5.0));
  myData.setSamples(samples);
  curve1->setData(&myData);
  
  
//   double x[4];
//   double y[4];
//   x[0] = 0; y[0] = 0;
//   x[1] = 1; y[1] = 1;
//   x[2] = 2; y[2] = 2;
//   x[3] = 3; y[3] = 4;
//   curve1->setRawSamples(x, y, 4);
  
  curve1->attach(myPlot);
  
  myPlot->replot();
}
