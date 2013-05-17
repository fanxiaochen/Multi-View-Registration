#pragma once
#ifndef FILE_VIEWER_WIDGET_H
#define FILE_VIEWER_WIDGET_H

#include <QTreeView>

class FileSystemModel;

class FileViewerWidget : public QTreeView
{
public:
  FileViewerWidget(QWidget * parent = 0);
  virtual ~FileViewerWidget(void);

  virtual QSize sizeHint() const {return QSize(320, 480);}
  void setWorkspace(const std::string& workspace);
  FileSystemModel* getFileSystemModel() {return model_;}

protected:
  virtual void contextMenuEvent(QContextMenuEvent *event);

private:
  FileSystemModel*  model_;
};

#endif /*FILE_VIEWER_WIDGET_H*/