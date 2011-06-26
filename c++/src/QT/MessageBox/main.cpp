#include <QApplication>

#include "messagebox.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MyForm form;
    
    form.show();
    return app.exec();
}
