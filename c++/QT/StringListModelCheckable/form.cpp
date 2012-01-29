#include <QStringListModel>
#include <QtGui>

#include <iostream>

#include "form.h"

// 3 options to extend this to have two bool members per item:
// 1) either you find a proper storage and build two models on top of it (which actually could be one model with a flag...);
// 2) use two roles and a custom delegate (at least) one of the views, so that the custom role is rendered and acts as a checkbox;
// 3) use two roles and put a proxy model between the model and (at least) one of the views, the proxy model translating from/to your custom role to the Qt::CheckStateRole (peppe suggests this one)

class CheckableStringListModel : public QStringListModel
{
  
public:
  void setMyStringList(const QStringList &strings)
  {
    emit beginResetModel();
    CheckedStatus.resize(strings.size());
    for(unsigned int i = 0; i < CheckedStatus.size(); ++i)
      {
      CheckedStatus[i] = Qt::Unchecked;
      }
    setStringList(strings);
    emit endResetModel();
  }

private:
  QVector<Qt::CheckState> CheckedStatus;
  
  Qt::ItemFlags flags(const QModelIndex& index) const
  {
    // If the index is not valid
    if (index.row() < 0 || index.row() >= rowCount())
      {
      return Qt::NoItemFlags;
      }

    return Qt::ItemIsEditable | Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsUserCheckable;
  }

  QVariant data (const QModelIndex  &index , int role ) const
  {
    // From QStringListModel 
    if (index.row() < 0 || index.row() >= rowCount())
      {
      return QVariant();
      }

    if (role == Qt::DisplayRole || role == Qt::EditRole)
      {
      return stringList().at(index.row());
      }

    if(role == Qt::CheckStateRole)
      {
      return CheckedStatus[index.row()];
      }

    return QVariant();
  }

  bool setData (const QModelIndex &index, const QVariant &value, int role)
  {
    // From QStringListModel
    if (index.row() < 0 || index.row() >= rowCount())
      {
      return false;
      }

    if(role == Qt::EditRole || role == Qt::DisplayRole)
      {
      stringList().replace(index.row(), value.toString());
      emit dataChanged(index, index);
      return true;
      }
    if(role == Qt::CheckStateRole)
      {
      //CheckedStatus[index.row()] = value.value<Qt::CheckState>();
      CheckedStatus[index.row()] = static_cast<Qt::CheckState>(value.toUInt());
    
      emit dataChanged(index, index);
      }
    return true;

  }
};

Form::Form(QWidget *parent) : QWidget(parent)
{
  setupUi(this);

  CheckableStringListModel* model = new CheckableStringListModel;

  QStringList list;
  list << "a" << "b" << "c";
  model->setMyStringList(list);
  this->listView->setModel(model);
}
