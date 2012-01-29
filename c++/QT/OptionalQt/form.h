#ifndef FORM_H
#define FORM_H

#if defined(INTERACTIVE)
#warning "Using interactive!"
#include <QWidget>
class Form : public QWidget
#else
#warning "Not using interactive!"
class Form
#endif
{
  #if defined(INTERACTIVE)
    Q_OBJECT
  
public:
    
  
  public slots:
    
  #endif
};

#endif
