#ifndef CustomListView_H
#define CustomListView_H

#include <QListView>

class QStringListModel;

class CustomListView : public QListView
{
  Q_OBJECT

public:
  CustomListView(QWidget *parent = 0);

};

/*
// Dummy Application Entity
#include <QStringList>
typedef QStringList EntityList;

#include <QTreeView>
#include <QAbstractItemModel>
#include <QStyledItemDelegate>

class QAction;
class QIcon;
class QMenu;
class QMouseEvent;

// ******************************************
// ***  class BoilerTreeView : QTreeView  ***
// ******************************************

class BoilerTreeView : public QTreeView
{
  Q_OBJECT

  private:
   class ItemModel;      // declared below, a QAbstractItemModel
   class ItemDelegate;   // declared below, a QStyledItemDelegate

  private:
   ItemModel*    _itemModel;
   ItemDelegate* _itemDelegate;

   QMenu* _contextMenu;
   int _contextRow;
   int _contextCol;
   QAction* _appendEntity_CtxAction;
   QAction* _removeEntity_CtxAction;

   bool _clientSetEntityList_inProgress;

  public:
   BoilerTreeView (QWidget* parentWid);
   virtual ~BoilerTreeView();

   void setEntityList (const EntityList&);
   const EntityList& entityListRef() const;
   void appendEntityItem (const QString& entityName);
   void showRemoveColumn (bool);

   // notification from internal ItemModel class
   void processModelDataChanged();

  private:
   void initContextMenu();
   void initConnections();
   void openEntity (int row);
   void removeEntityItem (int row);

  private slots:
   void contextMenuRequested (const QPoint&);
   void appendEntity_ctxTriggered();
   void removeEntity_ctxTriggered();
   void cell_clicked (const QModelIndex&);
   void cell_doubleClicked (const QModelIndex&);
   void cell_entered (const QModelIndex&);
   void cell_pressed (const QModelIndex&);

  protected:
   // virtual from QWidget
   virtual void mousePressEvent (QMouseEvent*);
   virtual void mouseReleaseEvent (QMouseEvent*);

  signals:
   void entityListEdited();
};

// **************************************************************
// ***  class BoilerTreeView::ItemModel : QAbstractItemModel  ***
// **************************************************************

class BoilerTreeView::ItemModel : public QAbstractItemModel
{
  Q_OBJECT

  private:
   BoilerTreeView* _parentTreeView;
   EntityList _entityList;

   mutable QIcon* _entityIcon;
   mutable QSize  _entityIconSize;
   mutable QIcon* _pencilIcon;
   mutable QSize  _pencilIconSize;
   mutable QIcon* _ellipsisIcon;
   mutable QSize  _ellipsisIconSize;
   mutable QIcon* _removeIcon;
   mutable QSize  _removeIconSize;

  public:
   ItemModel (BoilerTreeView* parentTreeView);
   virtual ~ItemModel();

   void setModelEntityList (const EntityList&);
   const EntityList& modelEntityListRef() const { return _entityList; }

   // returns row index of new item
   int appendEntityItem (const QString& entityName);

  private:
   QString cellStr (int row, int col) const;
   const QIcon& cellIcon (int row, int col, QSize& retSize) const;
   QSize cellSize (int row, int col) const;

   const QIcon& entityIcon() const;    // instantiate if necessary.
   const QIcon& pencilIcon() const;    // instantiate if necessary.
   const QIcon& ellipsisIcon() const;  // instantiate if necessary.
   const QIcon& removeIcon() const;    // instantiate if necessary.

  public:
   QSize maxIconSize() const;

  public:
   // ***  Virtual methods from QAbstractItemModel  ***
   Qt::ItemFlags flags (const QModelIndex&) const;
   QModelIndex index (int row, int col,
                     const QModelIndex& parentInx=QModelIndex()) const;

   QModelIndex parent (const QModelIndex&) const { return QModelIndex(); }
   int rowCount (const QModelIndex& parentInx=QModelIndex()) const;
   int columnCount (const QModelIndex& parentInx=QModelIndex()) const;

   QVariant data (const QModelIndex&, int role=Qt::DisplayRole) const;
   QVariant headerData (int section, Qt::Orientation,
                                            int role=Qt::DisplayRole) const;

   bool setData (const QModelIndex&, const QVariant&, int role=Qt::EditRole);

   // -------------------------------------------------
};

// ******************************************************************
// ***  class BoilerTreeView::ItemDelegate : QStyledItemDelegate  ***
// ******************************************************************

class BoilerTreeView::ItemDelegate : public QStyledItemDelegate
{
  Q_OBJECT

  private:
   BoilerTreeView* _parentTreeView;

  public:
   ItemDelegate (BoilerTreeView* parentTreeView);
   virtual ~ItemDelegate();

   // ***  Virtual methods from QAbstractItemDelegate  ***

   // Note: when defering to the base class implementation, make sure
   // to call the direct base class method, i.e. of QStyledItemDelegate.

   QWidget* createEditor (QWidget* parentWid, const QStyleOptionViewItem&,
                          const QModelIndex&) const;

   void setEditorData (QWidget* editor, const QModelIndex&) const;

   void setModelData (QWidget* editor, QAbstractItemModel* itemModel,
                      const QModelIndex&) const;

   // ----------------------------------------------------
};

// ********************************************
// ***  class BoilerTreeViewTest : QWidget  ***
// ********************************************

#include <QWidget>

class QHBoxLayout;
class QLabel;
class QPushButton;
class QVBoxLayout;

class BoilerTreeViewTest : public QWidget
{
  Q_OBJECT

  private:
    QVBoxLayout*     _mainVBox;
    BoilerTreeView*  _boilerTreeView;
    QHBoxLayout*     _buttonHBox;
    QPushButton*     _unlockButton;
    QPushButton*     _addEntityButton;
    QLabel*          _addEntityTitleLabel;

  public:
    BoilerTreeViewTest (QWidget* parentWid);

  private slots:
    void boilerTreeView_entityListEdited();
    void unlock_clicked();
    void addEntity_clicked();

  private:
    void updateButtons();
};*/

#endif
