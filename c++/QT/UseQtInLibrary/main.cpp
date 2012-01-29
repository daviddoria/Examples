#include <QApplication>

#ifdef QT_VERSION
  #pragma message("Main Using Qt!")
#else
  #pragma message("Main NOT using Qt!")
#endif

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    return 0;
}
