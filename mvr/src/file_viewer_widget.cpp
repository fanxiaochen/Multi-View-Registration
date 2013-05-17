#include <QMenu>
#include <QContextMenuEvent>

#include "main_window.h"
#include "point_cloud.h"
#include "parameter_manager.h"
#include "file_system_model.h"
#include "file_viewer_widget.h"


FileViewerWidget::FileViewerWidget(QWidget * parent)
  : QTreeView(parent),
  model_(new FileSystemModel)
{
  setModel(model_);

  setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

  hideColumn(1);
  hideColumn(2);
  hideColumn(3);
}

FileViewerWidget::~FileViewerWidget(void)
{
}

void FileViewerWidget::setWorkspace(const std::string& workspace)
{
  QModelIndex root_index = model_->setRootPath(workspace.c_str());
  setRootIndex(root_index);
  ParameterManager::getInstance().initObjectNumbers();

  return;
}

void FileViewerWidget::contextMenuEvent(QContextMenuEvent *event)
{
  std::string filename = model_->filePath(currentIndex()).toStdString();
  osg::ref_ptr<PointCloud> point_cloud = model_->getPointCloud(filename);
  if (point_cloud == NULL)
    return;

  QMenu menu(this);

  int view = point_cloud->getView();

  if (view != 12)
  {
    menu.addAction("Set Cloud Rotation", point_cloud, SLOT(setRotation()));
    menu.addAction(QString("Mark as %1").arg(point_cloud->isRegistered()?("Unregistered"):("Registered")), point_cloud, SLOT(toggleRegisterState()));
  }
   
  menu.exec(event->globalPos());

  return;
}