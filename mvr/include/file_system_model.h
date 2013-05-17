#pragma once
#ifndef FILE_SYSTEM_MODEL_H
#define FILE_SYSTEM_MODEL_H

#include <unordered_map>

#include <QSet>
#include <QHash>
#include <QMutex>
#include <QFileSystemModel>
#include <QPersistentModelIndex>

#include <osg/ref_ptr>
#include <osg/Vec4>

class PointCloud;

class FileSystemModel : public QFileSystemModel
{
  Q_OBJECT

public:
  FileSystemModel();
  virtual ~FileSystemModel();

  QModelIndex setRootPath ( const QString & newPath );

  Qt::ItemFlags flags(const QModelIndex &index) const;
  QVariant data(const QModelIndex &index, int role) const;
  bool setData(const QModelIndex &index, const QVariant &value, int role);
  bool isShown(const std::string& filename) const;

  osg::ref_ptr<PointCloud> getPointCloud(const std::string& filename);
  osg::ref_ptr<PointCloud> getPointCloud(int object);   
  osg::ref_ptr<PointCloud> getPointCloud(int object, int view);
  void getObjectRange(int &start, int &end);
  std::string getPointsFolder(int object);
  std::string getPointsFolder(int object, int view);

  std::string getPointsFilename(int object, int view);
  std::string getPointsFilename(int object);

  void showPointCloud(const std::string& filename);

  void hidePointCloud(int object, int view);
  void hidePointCloud(const std::string& filename);

  void updatePointCloud(int object, int view);
  void updatePointCloud(int object);
  const osg::Vec4& getColor(void) const {return color_;}

  int getStartObject(void) const {return start_object_;}
  int getEndObject(void) const {return end_object_;}
  PointCloud* getDisplayFirstObject(void);

  enum NavigationType
  {
    SWITCH,
    APPEND,
    ERASE
  };
  void navigateToPreviousObject(NavigationType type);
  void navigateToNextObject(NavigationType type);
  void navigateToPreviousView(NavigationType type);
  void navigateToNextView(NavigationType type);

  public slots:

    void showPointCloud(int object, int view);
    void hideAndShowPointCloud(int hide_object, int hide_view, int show_object, int show_view);

signals:
    void progressValueChanged(int value);
    void timeToHideAndShowPointCloud(int hide_object, int hide_view, int show_object, int show_view);
    void timeToShowPointCloud(int show_object, int show_view);

private:
  
  Qt::CheckState computeCheckState(const QModelIndex &index) const;
  bool checkRegisterState(const QModelIndex &index) const;
  
  
  void showPointCloud(const QPersistentModelIndex& index);
  void hidePointCloud(const QPersistentModelIndex& index);
  void showPointCloudSceneInformation(void) const;
  
  
  void limitPointCloudCacheSize(void);

  void getDisplayFirstObjectFirstView(int& object, int& view);
  void getDisplayFirstObjectLastView(int& object, int& view);
  void getDisplayLastObjectLastView(int& object, int& view);

  bool recursiveCheck(const QModelIndex &index, const QVariant &value);
  void computeObjectRange(void);

private:

  typedef QHash<QPersistentModelIndex, osg::ref_ptr<PointCloud> > PointCloudMap;
  typedef std::unordered_map<std::string, osg::ref_ptr<PointCloud> > PointCloudCacheMap;

  QSet<QPersistentModelIndex>     checked_indexes_;
  PointCloudMap                   point_cloud_map_;
  PointCloudCacheMap              point_cloud_cache_map_;
  int                             start_object_;
  int                             end_object_;
  osg::Vec4                       color_;
  QMutex                          mutex_;
};
#endif // FILE_SYSTEM_MODEL_H
