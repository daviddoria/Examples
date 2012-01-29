#include <QLineEdit>

#ifdef QT_VERSION
  #pragma message("Lib Using Qt!")
#else
  #pragma message("Lib NOT using Qt!")
#endif


void myfunction()
{
#ifdef QT_VERSION
  #pragma message("myfunction Using Qt!")
#else
  #pragma message("myfunction NOT using Qt!")
#endif
}
