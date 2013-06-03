#pragma once
#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <QObject>
#include <QColor>
#include <osg/Array>
#include <unordered_set>

#include <osg/MatrixTransform>
#include <QMutex>

#include "Renderable.h"
#include "types.h"

namespace osgManipulator
{
  class TranslateAxisDragger;
  class TrackballDragger;
}


class PointCloud : public QObject, public Renderable, public PCLRichPointCloud
{
  Q_OBJECT

public:
  PointCloud(void);
  virtual ~PointCloud(void);

  virtual const char* className() const {return "PointCloud";}

  bool open(const std::string& filename);
  bool save(const std::string& filename);
  void reload(void);

  inline const std::string& getFilename(void) const {return filename_;}

  void getTransformedPoints(PCLPointCloud& points);

  inline bool isRegistered(void) const {return registered_;}
  void setRegisterState(bool registered);

  void registration(int segment_threshold, int max_iterations, double max_distance);
//  void denoise(int segment_threshold, double triangle_length);

  int getObject(void) const;
  int getView(void) const;
  bool isShown(void) const;
  void initRotation(void);

  public slots:
    void setRotation(void);
    void registration(void);

    void toggleDraggers(void);
    void toggleRegisterState(void);

    void save(void);

protected:
  virtual void clearData();
  virtual void updateImpl();

  PointCloud* getPrevObject(void);
  PointCloud* getNextObject(void);

  
//  void initPointGraph(double distance_threshold);
//  void triangulate(void);

  void loadTransformation(void);
  void saveTransformation(void);
  void deleteTransformation(void);

  void visualizePoints(size_t start, size_t end);

protected:
  std::string                     filename_;
  size_t                          points_num_;
  size_t                          noise_points_num_;
  osg::Vec4                       color_;

//  boost::PointGraph*              point_graph_;
//  double                          point_graph_threshold_;
//  mutable CGAL::Delaunay*         triangulation_;

private:
  osg::ref_ptr<osgManipulator::TranslateAxisDragger>  translate_dragger_;
  osg::ref_ptr<osgManipulator::TrackballDragger>      trackball_dragger_;

  QMutex                          mutex_;


  bool                            show_draggers_;
  bool                            registered_;
};

#endif // POINTCLOUD_H