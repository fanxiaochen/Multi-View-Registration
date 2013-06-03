#pragma once
#ifndef REGISTRATOR_H
#define REGISTRATOR_H

#include <QObject>
#include "renderable.h"

namespace osgManipulator
{
  class TranslateAxisDragger;
  class TrackballDragger;
}

class Registrator : public QObject, public Renderable
{
  Q_OBJECT
public:
  Registrator(void);
  virtual ~Registrator(void);

  virtual const char* className() const {return "Registrator";}

  void init(void);
  void reset(void);

  osg::Vec3 getPivotPoint() const;
  osg::Vec3 getAxisNormal() const;
  osg::Matrix getRotationMatrix(double angle) const;

  void setPivotPoint(const osg::Vec3& pivot_point);
  void setAxisNormal(const osg::Vec3& axis_normal);
  void save(int object);
  void load(int object);
  virtual void toggleRendering(void);

  void saveRegisteredPoints(int object);
  void refineAxis(int object);
  void registrationLUM(int segment_threshold, int max_iterations, double max_distance, int object);
  void registrationICP(int max_iterations, double max_distance, int object);
  void registrationICP(int max_iterations, double max_distance, int object, int repeat_times);

  bool isAxisAccurate();

  public slots:
    void load(void);
    void save(void);
    void saveRegisteredPoints(void);
    void refineAxis(void);
    void registrationICP(void);
    void registrationLUM(void);
    void automaticRegistration(void);

protected:
  virtual void clear();
  virtual void updateImpl();
  void computeError(int object);
  void visualizeError(void);
  void visualizeAxis(void);
  void save(const QString& filename);
  void load(const QString& filename);

protected:
  osg::ref_ptr<osg::MatrixTransform>                  pivot_point_;
  osg::ref_ptr<osg::MatrixTransform>                  normal_point_;
  osg::ref_ptr<osg::MatrixTransform>                  visualization_;
  osg::ref_ptr<osgManipulator::TranslateAxisDragger>  pivot_dragger_;
  osg::ref_ptr<osgManipulator::TrackballDragger>      normal_dragger_;

protected:
  osg::ref_ptr<osg::Vec3Array>  error_vertices_;
  osg::ref_ptr<osg::Vec4Array>  error_colors_;

private:
  bool              initilized_;
  bool              show_axis_;
  bool              show_error_;
};

#endif // REGISTRATOR_H