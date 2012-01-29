#include "MyTableModel.h"

#include <iostream>
#include <sstream>

#include <QLabel>
#include <QPixmap>

MyTableModel::MyTableModel() : QAbstractTableModel()
{

}

int MyTableModel::rowCount(const QModelIndex& parent) const
{
  return 5;
}

int MyTableModel::columnCount(const QModelIndex& parent) const
{
  return 2;
}

QVariant MyTableModel::data(const QModelIndex& index, int role) const
{
  if(role == Qt::DisplayRole)
    {
    if(index.column() == 0)
      {
      return index.row();
      }
    else if(index.column() == 1)
      {
      //std::cout << "Creating image." << std::endl;
      QPixmap pixmap(20,20);
      QColor black(0,0,0);
      pixmap.fill(black);
      return pixmap;

//       QLabel* imageLabel = new QLabel;
//       imageLabel->setPixmap(pixmap);
//       return imageLabel; // Seems to not be able to be casted to QVariant
      }
    }
    // This works, but doesn't allow you to resize the image
  else if(role == Qt::DecorationRole && index.column() == 1)
    {
    int imageSize = 100;
    QPixmap pixmap(imageSize, imageSize);
    QColor black(0,0,0);
    pixmap.fill(black);
    return pixmap;
    }
  else if (role == Qt::TextAlignmentRole && index.column() == 1)
    {
    return Qt::AlignCenter;
    }
//   else if (role == Qt::alignmen && index.column() == 1)
//     {
//     return Qt::AlignCenter;
//     }

  return QVariant::Invalid;
}

Qt::ItemFlags MyTableModel::flags(const QModelIndex & index ) const
{
  return Qt::ItemIsEditable | Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}
