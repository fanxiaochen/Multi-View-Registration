#pragma once
#ifndef REGISTRATOR_H
#define REGISTRATOR_H

#include <QObject>
#include <pcl/registration/icp.h>

#include "types.h"
#include "renderable.h"
#include "point_cloud.h"

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

  void saveRegisteredPoints(int object, int segment_threshold);
  void refineAxis(int object);
  void registrationLUM(int segment_threshold, int max_iterations, double max_distance, int object);
  void registrationICP(int max_iterations, double max_distance, int object);
  void registrationICP(int max_iterations, double max_distance, int object, int repeat_times);
  void registration(int object, int segment_threshold);

<<<<<<< HEAD
  void automaticRegistration(int object, int segment_threshold, int max_iterations, int repeat_times, double max_distance, 
    double transformation_epsilon, double euclidean_fitness_epsilon);
=======
  void automaticRegistration(int object, int segment_threshold, int max_iterations, 
    int repeat_times, double max_distance,  double transformation_epsilon, double euclidean_fitness_epsilon);
>>>>>>> 37b83026ef9413c4908571d01b42f5607391064f

  public slots:
    void load(void);
    void save(void);
    void refineAxis(void);
    void registrationICP(void);
    void registrationLUM(void);
    void registration(void);
    void automaticRegistration(void);

protected:
  virtual void clear();
  virtual void updateImpl();
  void computeError(int object);
  void visualizeError(void);
  void visualizeAxis(void);
  void save(const QString& filename);
  void load(const QString& filename);
<<<<<<< HEAD
  void setCriteria(int source_number);
  void addEuclideanFitnessEpsilon(double euclidean_fitness_epsilon);
  void automaticRegistrationICP(int view_number, int object, int max_iterations, int repeat_times, double max_distance, 
    double transformation_epsilon, double euclidean_fitness_epsilon);
  void refineTransformation(int repeat_times, int source_index);
=======
  void automaticRefineTransformation(int max_iterations, size_t source_index);

>>>>>>> 37b83026ef9413c4908571d01b42f5607391064f

protected:
  osg::ref_ptr<osg::MatrixTransform>                  pivot_point_;
  osg::ref_ptr<osg::MatrixTransform>                  normal_point_;
  osg::ref_ptr<osg::MatrixTransform>                  visualization_;
  osg::ref_ptr<osgManipulator::TranslateAxisDragger>  pivot_dragger_;
  osg::ref_ptr<osgManipulator::TrackballDragger>      normal_dragger_;

protected:
  osg::ref_ptr<osg::Vec3Array>  error_vertices_;
  osg::ref_ptr<osg::Vec4Array>  error_colors_;

  pcl::IterativeClosestPoint<PCLPoint, PCLPoint>   icp_;
  std::vector<osg::ref_ptr<PointCloud> > point_clouds_;
  PCLPointCloud::Ptr source_;
  PCLPointCloud::Ptr target_;
<<<<<<< HEAD
=======
  
>>>>>>> 37b83026ef9413c4908571d01b42f5607391064f

private:
  bool              initilized_;
  bool              show_axis_;
  bool              show_error_;
};

#endif // REGISTRATOR_H