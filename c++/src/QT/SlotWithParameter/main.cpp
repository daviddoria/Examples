#include <QApplication>

#include "form.h"

/* this doesn't work */

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MyForm form;

    form.show();
    return app.exec();
}
