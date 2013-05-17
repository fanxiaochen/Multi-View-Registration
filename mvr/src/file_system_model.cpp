#include <QDir>
#include <QColor>
#include <QMutexLocker>
#include <QColorDialog>
#include <QFutureWatcher>
#include <QtConcurrentRun>

#include <osg/Group>
#include <osgDB/ReadFile>

#include "main_window.h"
#include "point_cloud.h"
#include "parameter_manager.h"
#include "osg_viewer_widget.h"
#include "file_system_model.h"


FileSystemModel::FileSystemModel()
  :start_object_(-1),
  end_object_(-1)
{
  setNameFilterDisables(false);
  QStringList allowed_file_extensions;
  allowed_file_extensions.push_back("*.pcd");
  allowed_file_extensions.push_back("*.obj");     
  setNameFilters(allowed_file_extensions);

  connect(this, SIGNAL(timeToHideAndShowPointCloud(int, int, int, int)), this, SLOT(hideAndShowPointCloud(int, int, int, int)));
  connect(this, SIGNAL(timeToShowPointCloud(int, int)), this, SLOT(showPointCloud(int, int)));
}

FileSystemModel::~FileSystemModel()
{
}

Qt::ItemFlags FileSystemModel::flags(const QModelIndex &index) const
{
  return QFileSystemModel::flags(index) | Qt::ItemIsUserCheckable;
}

QVariant FileSystemModel::data(const QModelIndex &index, int role) const
{
  if(role == Qt::CheckStateRole)
    return computeCheckState(index);
  else
  {
    if(role == Qt::ForegroundRole && checkRegisterState(index))
      return QBrush(QColor(255, 0, 0));
    return QFileSystemModel::data(index, role);
  }
}

bool FileSystemModel::checkRegisterState(const QModelIndex &index) const
{
  PointCloudCacheMap::const_iterator it = point_cloud_cache_map_.find(filePath(index).toStdString());
  if (it != point_cloud_cache_map_.end())
    return it->second.get()->isRegistered();
  else
    return false;
}

Qt::CheckState FileSystemModel::computeCheckState(const QModelIndex &index) const
{
  if(!hasChildren(index))
    return (checked_indexes_.contains(index)) ? (Qt::Checked) : (Qt::Unchecked);

  bool all_checked = true;
  bool all_unchecked = true;
  for(int i = 0, i_end = rowCount(index); i < i_end; i ++)
  {
    QModelIndex child = QFileSystemModel::index(i, 0, index);
    Qt::CheckState check_state = computeCheckState(child);
    if (check_state == Qt::PartiallyChecked)
      return check_state;

    if (check_state == Qt::Checked)
      all_unchecked = false;
    if (check_state == Qt::Unchecked)
      all_checked = false;

    if (!all_checked && !all_unchecked)
      return Qt::PartiallyChecked;
  }

  if (all_unchecked)
    return Qt::Unchecked;

  return Qt::Checked;
}

bool FileSystemModel::isShown(const std::string& filename) const
{
  QModelIndex index = this->index(filename.c_str());
  return (checked_indexes_.contains(index)) ? (true) : (false);
}

bool FileSystemModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
  bool is_point_cloud = (filePath(index).right(3) == "pcd");
  if(role == Qt::CheckStateRole)
  {
    if (is_point_cloud)
    {
      if(value == Qt::Checked)
        showPointCloud(index);
      else
        hidePointCloud(index);
    }

    if(hasChildren(index) == true)
      recursiveCheck(index, value);

    emit dataChanged(index, index);
    return true;
  }

  return QFileSystemModel::setData(index, value, role);
}

void FileSystemModel::limitPointCloudCacheSize(void)
{
  size_t threshold = 32;

  if (point_cloud_cache_map_.size() <= threshold)
    return;

  std::set<osg::ref_ptr<PointCloud> > in_use_clouds;
  std::vector<osg::ref_ptr<PointCloud> > freeable_clouds;

  for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
    in_use_clouds.insert(*it);

  for (PointCloudCacheMap::const_iterator it = point_cloud_cache_map_.begin(); it != point_cloud_cache_map_.end(); ++ it)
    if (in_use_clouds.find(it->second) == in_use_clouds.end() && it->second->referenceCount() == 1)
      freeable_clouds.push_back(it->second);

  for (size_t i = 0, i_end = freeable_clouds.size(); i < i_end; ++ i)
    point_cloud_cache_map_.erase(freeable_clouds[i]->getFilename());

  return;
}

osg::ref_ptr<PointCloud> FileSystemModel::getPointCloud(const std::string& filename)
{
  QMutexLocker locker(&mutex_);

  limitPointCloudCacheSize();

  QFileInfo fileinfo(filename.c_str());
  if (!fileinfo.exists() || !fileinfo.isFile())
    return NULL;

  PointCloudCacheMap::iterator it = point_cloud_cache_map_.find(filename);
  if (it != point_cloud_cache_map_.end())
    return it->second.get();

  osg::ref_ptr<PointCloud> point_cloud(new PointCloud());
  if (!point_cloud->open(filename))
    return NULL;

  if (point_cloud->thread() != qApp->thread())
    point_cloud->moveToThread(qApp->thread());

  point_cloud_cache_map_[filename] = point_cloud;

  return point_cloud;
}

QModelIndex FileSystemModel::setRootPath ( const QString & newPath )
{
  point_cloud_cache_map_.clear();
  point_cloud_map_.clear();
  checked_indexes_.clear();

  QModelIndex index = QFileSystemModel::setRootPath(newPath);
  computeObjectRange();
  if (start_object_ != -1)
  {
    if (getPointCloud(start_object_) != NULL)
      showPointCloud(start_object_, 12);
    else if (getPointCloud(start_object_, 0) != NULL)
      showPointCloud(start_object_, 0);

    MainWindow::getInstance()->getOSGViewerWidget()->centerScene();
  }

  return index;
}

static void extractStartEndObject(const QStringList& entries, int& start_object, int& end_object)
{
  start_object = std::numeric_limits<int>::max();
  end_object = std::numeric_limits<int>::min();

  for (QStringList::const_iterator entries_it = entries.begin();
    entries_it != entries.end(); ++ entries_it)
  {
    if (!entries_it->contains("object_"))
      continue;

    int index = entries_it->right(4).toInt();
    if (start_object > index)
      start_object = index;
    if (end_object < index)
      end_object = index;
  }

  return;
}

void FileSystemModel::computeObjectRange(void)
{
  start_object_ = end_object_ = -1;

  QString root_path = rootPath();
  QModelIndex root_index = index(root_path);

  if (root_path.contains("object_")) {
    start_object_ = end_object_ = root_path.right(4).toInt();
    return;
  }

  if (root_path.compare("points") == 0)
  {
    QStringList points_entries = QDir(root_path).entryList();
    extractStartEndObject(points_entries, start_object_, end_object_);
    return;
  }

  QStringList root_entries = QDir(root_path).entryList();
  for (QStringList::const_iterator root_entries_it = root_entries.begin();
    root_entries_it != root_entries.end(); ++ root_entries_it)
  {
    if (root_entries_it->compare("points") != 0)
      continue;

    QStringList points_entries = QDir(root_path+"/"+*root_entries_it).entryList();
    extractStartEndObject(points_entries, start_object_, end_object_);
    return;
  }

  return;
}

void FileSystemModel::getObjectRange(int &start, int &end)
{
  start = start_object_;
  end = end_object_;
}

osg::ref_ptr<PointCloud> FileSystemModel::getPointCloud(int object)
{
  return getPointCloud(getPointsFilename(object));
}

std::string FileSystemModel::getPointsFolder(int object)
{
  QModelIndex root_index = index(rootPath());

  int start, end;
  getObjectRange(start, end);
  if (start < 0 || end < 0)
    return std::string();

  std::string folder;

  QString root_path = rootPath();
  if (root_path.contains("object_"))
  {
    folder =  root_path.toStdString();
  }
  else if (root_path.compare("points") == 0)
  {
    QModelIndex object_index = index(object-start, 0, root_index);
    folder = filePath(object_index).toStdString();
  }
  else
  {
    QStringList root_entries = QDir(root_path).entryList();
    for (QStringList::const_iterator root_entries_it = root_entries.begin();
      root_entries_it != root_entries.end(); ++ root_entries_it)
    {
      if (root_entries_it->compare("points") != 0)
        continue;

      folder = (root_path+QString("/%1/object_%2").arg(*root_entries_it).arg(object, 5, 10, QChar('0'))).toStdString();
      break;
    }
  }

  return folder;
}

std::string FileSystemModel::getPointsFolder(int object, int view)
{
  std::string object_folder = getPointsFolder(object);
  if (object_folder.empty() || view < 0 || view >= 12)
    return object_folder;

  QString view_folder = QString("%1/view_%2").arg(object_folder.c_str()).arg(view, 2, 10, QChar('0'));
  if (!QDir(view_folder).exists())
    view_folder = QString("%1/slice_%2").arg(object_folder.c_str()).arg(view, 2, 10, QChar('0'));

  return view_folder.toStdString();
}

std::string FileSystemModel::getPointsFilename(int object, int view)
{
  std::string folder = getPointsFolder(object, view);
  if (folder.empty())
    return folder;

  return folder+"/points.pcd";
}

std::string FileSystemModel::getPointsFilename(int object)
{
  return getPointsFilename(object, 12);
}

osg::ref_ptr<PointCloud> FileSystemModel::getPointCloud(int object, int view)
{
  return getPointCloud(getPointsFilename(object, view));
}

void FileSystemModel::showPointCloud(int object, int view)
{
  showPointCloud(getPointsFilename(object, view));
}

void FileSystemModel::showPointCloud(const std::string& filename)
{
  QModelIndex index = this->index(QString(filename.c_str()));
  if (!index.isValid())
    return;

  showPointCloud(index);

  return;
}

void FileSystemModel::showPointCloud(const QPersistentModelIndex& index)
{
  checked_indexes_.insert(index);

  osg::ref_ptr<PointCloud> point_cloud(getPointCloud(filePath(index).toStdString()));
  if (!point_cloud.valid())
    return;

  PointCloudMap::iterator point_cloud_map_it = point_cloud_map_.find(index);
  if (point_cloud_map_it != point_cloud_map_.end())
    return;

  MainWindow::getInstance()->getOSGViewerWidget()->addChild(point_cloud);
  point_cloud_map_[index] = point_cloud;

  showPointCloudSceneInformation();

  return;
}

void FileSystemModel::hidePointCloud(const std::string& filename)
{
  QModelIndex index = this->index(QString(filename.c_str()));
  if (!index.isValid())
    return;

  hidePointCloud(index);
}

void FileSystemModel::hidePointCloud(int object, int view)
{
  hidePointCloud(getPointsFilename(object, view));
}

void FileSystemModel::hidePointCloud(const QPersistentModelIndex& index)
{
  checked_indexes_.remove(index);

  PointCloudMap::iterator point_cloud_map_it = point_cloud_map_.find(index);
  if (point_cloud_map_it == point_cloud_map_.end())
    return;

  MainWindow::getInstance()->getOSGViewerWidget()->removeChild(point_cloud_map_it.value().get());
  point_cloud_map_.erase(point_cloud_map_it);

  showPointCloudSceneInformation();

  return;
}

void FileSystemModel::hideAndShowPointCloud(int hide_object, int hide_view, int show_object, int show_view)
{
  bool to_hide = true;
  bool to_show = true;

  osg::ref_ptr<PointCloud> show_cloud = getPointCloud(show_object, show_view);
  osg::ref_ptr<PointCloud> hide_cloud = getPointCloud(hide_object, hide_view);

  QModelIndex show_index = this->index(QString(getPointsFilename(show_object, show_view).c_str()));
  checked_indexes_.insert(show_index);
  PointCloudMap::iterator point_cloud_map_it = point_cloud_map_.find(show_index);
  if (point_cloud_map_it == point_cloud_map_.end())
  {
    if (show_cloud != NULL)
      point_cloud_map_[show_index] = show_cloud;
    else
      to_show = false;
  }
  else
    to_show = false;

  QModelIndex hide_index = this->index(QString(getPointsFilename(hide_object, hide_view).c_str()));
  checked_indexes_.remove(hide_index);
  point_cloud_map_it = point_cloud_map_.find(hide_index);
  if (point_cloud_map_it != point_cloud_map_.end())
    point_cloud_map_.erase(point_cloud_map_it);
  else
    to_hide = false;

  OSGViewerWidget* osg_viewer_widget = MainWindow::getInstance()->getOSGViewerWidget();
  if (to_hide && to_show)
    osg_viewer_widget->replaceChild(hide_cloud, show_cloud, true);
  else if (to_hide)
    osg_viewer_widget->removeChild(getPointCloud(hide_object, hide_view), true);
  else if (to_show)
    osg_viewer_widget->addChild(getPointCloud(show_object, show_view), true);

  showPointCloudSceneInformation();

  return;
}

void FileSystemModel::showPointCloudSceneInformation(void) const
{
  QString information("Displaying Point Cloud:\n");

  if (point_cloud_map_.empty())
  {
 //   MainWindow::getInstance()->getInformation()->setText(information.toStdString(), 20, 20);
    return;
  }

  std::vector<std::pair<int, int> > sorted_scene_info;
  for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
  {
    int object = (*it)->getObject();
    int view = (*it)->getView();
    sorted_scene_info.push_back(std::make_pair(object, view));
  }

  std::sort(sorted_scene_info.begin(), sorted_scene_info.end());
  for (size_t i = 0, i_end = sorted_scene_info.size(); i < i_end; ++ i)
  {
    int object = sorted_scene_info[i].first;
    int view = sorted_scene_info[i].second;
    if (view < 12)
      information += QString("object %1 View %2\n").arg(object, 5, 10, QChar('0')).arg(view, 2, 10, QChar('0'));
    else
      information += QString("object %1\n").arg(object, 5, 10, QChar('0'));
  }
//  MainWindow::getInstance()->getInformation()->setText(information.toStdString(), 20, 20);

  return;
}

PointCloud* FileSystemModel::getDisplayFirstObject(void)
{
  if (point_cloud_map_.empty())
    return NULL;

  osg::ref_ptr<PointCloud> first_cloud = *point_cloud_map_.begin();
  int object = first_cloud->getObject();
  int view = first_cloud->getView();

  for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
  {
    osg::ref_ptr<PointCloud> point_cloud = *it;
    int this_object = point_cloud->getObject();
    int this_view = point_cloud->getView();
    if (this_object < object)
    {
      object = this_object;
      view = this_view;
    }
    else if(this_object == object && this_view > view)
    {
      object = this_object;
      view = this_view;
    }
  }

  if (view != 12)
    return NULL;

  return getPointCloud(object);
}

void FileSystemModel::getDisplayFirstObjectFirstView(int& object, int& view)
{
  if (point_cloud_map_.empty())
  {
    object = -1;
    view = -1;
    return;
  }

  osg::ref_ptr<PointCloud> first_cloud = *point_cloud_map_.begin();
  object = first_cloud->getObject();
  view = first_cloud->getView();

  for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
  {
    osg::ref_ptr<PointCloud> point_cloud = *it;
    int this_object = point_cloud->getObject();
    int this_view = point_cloud->getView();
    if (this_object < object)
    {
      object = this_object;
      view = this_view;
    }
    else if(this_object == object && this_view < view)
    {
      object = this_object;
      view = this_view;
    }
  }
  return;
}

void FileSystemModel::getDisplayFirstObjectLastView(int& object, int& view)
{
  if (point_cloud_map_.empty())
  {
    object = -1;
    view = -1;
    return;
  }

  std::vector<std::pair<int, int> > display_items;
  for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
  {
    osg::ref_ptr<PointCloud> point_cloud = *it;
    display_items.push_back(std::make_pair(point_cloud->getObject(), point_cloud->getView()));
  }

  object = display_items[0].first;
  view = display_items[0].second;
  for (size_t i = 1, i_end = display_items.size(); i < i_end; ++ i)
  {
    int this_object = display_items[i].first;
    if (this_object > object)
      object = this_object;
  }
  for (size_t i = 1, i_end = display_items.size(); i < i_end; ++ i)
  {
    int this_object = display_items[i].first;
    int this_view = display_items[i].second;
    if (this_object == object && this_view > view)
      view = this_view;
  }
  return;
}

void FileSystemModel::getDisplayLastObjectLastView(int& object, int& view)
{
  if (point_cloud_map_.empty())
  {
    object = -1;
    view = -1;
    return;
  }

  osg::ref_ptr<PointCloud> first_cloud = *point_cloud_map_.begin();
  object = first_cloud->getObject();
  view = first_cloud->getView();

  for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
  {
    osg::ref_ptr<PointCloud> point_cloud = *it;
    int this_object = point_cloud->getObject();
    int this_view = point_cloud->getView();
    if (this_object > object)
    {
      object = this_object;
      view = this_view;
    }
    else if(this_object == object && this_view > view)
    {
      object = this_object;
      view = this_view;
    }
  }
  return;
}

bool FileSystemModel::recursiveCheck(const QModelIndex &index, const QVariant &value)
{
  if(!hasChildren(index))
    return false;

  for(int i = 0, i_end = rowCount(index); i < i_end; i ++)
  {
    QModelIndex child = QFileSystemModel::index(i, 0, index);
    setData(child, value, Qt::CheckStateRole);
  }

  return true;
}

void FileSystemModel::updatePointCloud(int object, int view)
{
  std::string filename = getPointsFilename(object, view);
  if (point_cloud_cache_map_.find(filename) == point_cloud_cache_map_.end())
    return;

  getPointCloud(object, view)->reload();

  return;
}

void FileSystemModel::updatePointCloud(int object)
{
  std::string filename = getPointsFilename(object);
  if (point_cloud_cache_map_.find(filename) == point_cloud_cache_map_.end())
    return;

  getPointCloud(object)->reload();

  return;
}


void FileSystemModel::navigateToPreviousObject(NavigationType type)
{
  int first_object, first_view;
  getDisplayFirstObjectFirstView(first_object, first_view);

  if (first_object == -1 || first_view == -1)
  {
    showPointCloud(getStartObject(), 12);
    return;
  }

  if (type == ERASE)
  {
    hidePointCloud(first_object, first_view);
    return;
  }

  int current_object = first_object-1;
  if (current_object < getStartObject())
    return;

  if (type == APPEND)
    showPointCloud(current_object, first_view);
  else
    hideAndShowPointCloud(first_object, first_view, current_object, first_view);

  return;
}

void FileSystemModel::navigateToNextObject(NavigationType type)
{
  int last_object, last_view;
  getDisplayLastObjectLastView(last_object, last_view);

  if (last_object == -1 || last_view == -1)
  {
    showPointCloud(getEndObject(), 12);
    return;
  }

  if (type == ERASE)
  {
    hidePointCloud(last_object, last_view);
    return;
  }

  int current_object = last_object+1;
  if (current_object > getEndObject())
    return;

  if (type == APPEND)
    showPointCloud(current_object, last_view);
  else
    hideAndShowPointCloud(last_object, last_view, current_object, last_view);

  return;
}

void FileSystemModel::navigateToPreviousView(NavigationType type)
{
  int first_object, first_view;
  getDisplayFirstObjectFirstView(first_object, first_view);

  if (first_object == -1 || first_view == -1)
  {
    showPointCloud(getStartObject(), 0);
    return;
  }

  if (type == ERASE)
  {
    hidePointCloud(first_object, first_view);
    return;
  }

  int current_view = first_view-1;
  if (current_view < 0)
    return;

  if (type == APPEND)
    showPointCloud(first_object, current_view);
  else
    hideAndShowPointCloud(first_object, first_view, first_object, current_view);

  return;
}

void FileSystemModel::navigateToNextView(NavigationType type)
{
  int first_object, last_view;
  getDisplayFirstObjectLastView(first_object, last_view);

  if (first_object == -1 || last_view == -1)
  {
    showPointCloud(getStartObject(), 11);
    return;
  }

  if (type == ERASE)
  {
    hidePointCloud(first_object, last_view);
    return;
  }

  int current_view = last_view+1;
  if (current_view > 12)
    return;

  if (type == APPEND)
    showPointCloud(first_object, current_view);
  else
    hideAndShowPointCloud(first_object, last_view, first_object, current_view);

  return;
}
