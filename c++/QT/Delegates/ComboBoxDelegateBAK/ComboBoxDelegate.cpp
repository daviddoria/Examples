#include "ComboBoxDelegate.h"

#include <QComboBox>
#include <QWidget>
#include <QModelIndex>
#include <QApplication>
#include <QString>

#include <iostream>

ComboBoxDelegate::ComboBoxDelegate(QObject *parent)
:QItemDelegate(parent)
{
  Items.push_back("Test0");
  Items.push_back("Test1");
  Items.push_back("Test2");
  Items.push_back("Test3");
  Items.push_back("Test4");
  Items.push_back("Test5");
  Items.push_back("Test6");
  Items.push_back("Test7");
  Items.push_back("Test8");
}


QWidget *ComboBoxDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &/* option */, const QModelIndex &/* index */) const
{
  QComboBox* editor = new QComboBox(parent);
  for(unsigned int i = 0; i < Items.size(); ++i)
    {
    editor->addItem(Items[i].c_str());
    }
  return editor;
}

void ComboBoxDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
  int value = index.model()->data(index, Qt::EditRole).toUInt();
  QComboBox *comboBox = static_cast<QComboBox*>(editor);
  comboBox->setCurrentIndex(comboBox->findData(value));
}

void ComboBoxDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const
{
  QComboBox *comboBox = static_cast<QComboBox*>(editor);
  int value = comboBox->itemData(comboBox->currentIndex()).toUInt();
  model->setData(index, value, Qt::EditRole);
}

void ComboBoxDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &/* index */) const
{
  editor->setGeometry(option.rect);
}

void ComboBoxDelegate::addItem(const std::string& text, const QVariant& value)
{
  Items[value.toUInt()] = text;
}

void ComboBoxDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option,
                                                                const QModelIndex &index) const
{
  int value = index.data().toUInt();
  std::cout << "Paint: value: " << value << std::endl;
  QStyleOptionViewItemV4 myOption = option;
  QString text = Items[value].c_str();

  myOption.text = text;

  QApplication::style()->drawControl(QStyle::CE_ItemViewItem, &myOption, painter);

}
