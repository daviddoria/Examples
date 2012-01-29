#include <QApplication>

#include "form.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    Form myForm;
    myForm.show();
    return app.exec();
}
