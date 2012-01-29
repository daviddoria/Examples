#include "form.h"
//#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
itemWidth = 50;
itemHeight = 15;

srand((unsigned)time(NULL));

items = new QList<QGraphicsItem *>();

ui->setupUi(this);
connect(ui->addButton, SIGNAL(clicked()), this, SLOT(slot_addItem()));
scene = new QGraphicsScene(ui->canvas);
QBrush background = QBrush(Qt::red, Qt::SolidPattern);
scene->setBackgroundBrush(background);
scene->setSceneRect(0, 0, ui->centralWidget->width(), ui->centralWidget->height());
ui->canvas->setScene(scene);
}

MainWindow::~MainWindow()
{
delete ui;
}

void MainWindow::slot_addItem()
{
QPoint tl = QPoint(items->count() * itemWidth, items->count() * itemHeight);
QSize size = QSize(itemWidth, itemHeight);
QRect rectDims = QRect(tl, size);

DragableRect *item = new DragableRect();
item->setRect(rectDims);
item->show();

items->append(item);

scene->addItem(item);
}