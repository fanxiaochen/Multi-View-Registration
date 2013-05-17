#include <cstdio>

#include <QFileDialog>
#include <QMessageBox>
#include <QFutureWatcher>
#include <QtConcurrentRun>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/Geometry>
#include <osg/LineSegment>
#include <osg/ShapeDrawable>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/TranslateAxisDragger>

#include <boost/shared_ptr.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/lum.h>
#include <pcl/registration/correspondence_estimation.h>


#include "main_window.h"

#include "point_cloud.h"
#include "math_solvers.h"
#include "parameter_manager.h"
#include "osg_viewer_widget.h"
#include "file_system_model.h"
#include "osg_utility.h"

#include "registrator.h"

Registrator::Registrator(void)
  :pivot_point_(new osg::MatrixTransform),
  normal_point_(new osg::MatrixTransform),
  visualization_(new osg::MatrixTransform),
  pivot_dragger_(new osgManipulator::TranslateAxisDragger),
  normal_dragger_(new osgManipulator::TrackballDragger),
  initilized_(false),
  show_axis_(false),
  show_error_(false),
  error_vertices_(new osg::Vec3Array),
  error_colors_(new osg::Vec4Array)
{
  double point_radius = 2;
  osg::ref_ptr<osg::Sphere> sphere(new osg::Sphere(osg::Vec3(0, 0, 0), point_radius));

  osg::ref_ptr<osg::ShapeDrawable> pivot_point_drawable(new osg::ShapeDrawable(sphere.get()));
  pivot_point_drawable->setColor(osg::Vec4(0.0, 0.0, 0.5, 1.0));
  osg::ref_ptr<osg::Geode> pivot_point_geode = new osg::Geode;
  pivot_point_geode->addDrawable(pivot_point_drawable);
  pivot_point_->addChild(pivot_point_geode);

  osg::ref_ptr<osg::ShapeDrawable> normal_point_drawable(new osg::ShapeDrawable(sphere.get()));
  normal_point_drawable->setColor(osg::Vec4(0.5, 0.0, 0.0, 1.0));
  osg::ref_ptr<osg::Geode> normal_point_geode = new osg::Geode;
  normal_point_geode->addDrawable(normal_point_drawable);
  normal_point_->addChild(normal_point_geode);

  pivot_dragger_->setupDefaultGeometry();
  pivot_dragger_->setHandleEvents(true);
  pivot_dragger_->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
  pivot_dragger_->setActivationKeyEvent('d');

  normal_dragger_->setupDefaultGeometry();
  normal_dragger_->setHandleEvents(true);
  normal_dragger_->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
  normal_dragger_->setActivationKeyEvent('d');

  pivot_dragger_->addTransformUpdating(pivot_point_);
  pivot_dragger_->addTransformUpdating(normal_point_);
  pivot_dragger_->addTransformUpdating(visualization_);
  normal_dragger_->addTransformUpdating(pivot_point_);
  normal_dragger_->addTransformUpdating(normal_point_);
  normal_dragger_->addTransformUpdating(visualization_);

  pivot_dragger_->addTransformUpdating(normal_dragger_);
  normal_dragger_->addTransformUpdating(pivot_dragger_);

  pivot_point_->setMatrix(osg::Matrix::translate(osg::Vec3(0, 0, 0)));
  normal_point_->setMatrix(osg::Matrix::translate(osg::Vec3(0, -1, 0)));
}

Registrator::~Registrator(void)
{
}

osg::Vec3 Registrator::getPivotPoint() const
{
  return pivot_point_->getMatrix().getTrans();
}

osg::Vec3 Registrator::getAxisNormal() const
{
  osg::Vec3 pivot_point = pivot_point_->getMatrix().getTrans();
  osg::Vec3 normal_point = normal_point_->getMatrix().getTrans();
  osg::Vec3 axis_normal = normal_point-pivot_point;
  axis_normal.normalize();

  return axis_normal;
}

void Registrator::setPivotPoint(const osg::Vec3& pivot_point)
{
  osg::Vec3 old_pivot_point = pivot_point_->getMatrix().getTrans();
  osg::Vec3 old_normal_point = normal_point_->getMatrix().getTrans();
  osg::Vec3 axis_normal = old_normal_point-old_pivot_point;

  osg::Vec3 normal_point = pivot_point + axis_normal;

  pivot_point_->setMatrix(osg::Matrix::translate(pivot_point));
  normal_point_->setMatrix(osg::Matrix::translate(normal_point));

  expire();
}

void Registrator::setAxisNormal(const osg::Vec3& axis_normal)
{
  osg::Vec3 pivot_point = pivot_point_->getMatrix().getTrans();
  osg::Vec3 old_normal_point = normal_point_->getMatrix().getTrans();
  osg::Vec3 old_normal = old_normal_point-pivot_point;
  double length = old_normal.length();
  if (old_normal*axis_normal < 0)
    length = -length;

  osg::Vec3 normal = axis_normal;
  normal.normalize();

  osg::Vec3 normal_point = pivot_point + normal*length;
  normal_point_->setMatrix(osg::Matrix::translate(normal_point));

  expire();
}

void Registrator::reset(void)
{
  initilized_ = false;
  clear();

  return;
}


void Registrator::init(void)
{
  if (initilized_)
    return;

  osg::BoundingSphere boundingSphere = MainWindow::getInstance()->getOSGViewerWidget()->getBound();
  double radius = boundingSphere.radius();

  if (load())
  {
    normal_point_->setMatrix(osg::Matrix::translate(getPivotPoint()+getAxisNormal()*radius));
  }
  else
  {
    pivot_point_->setMatrix(osg::Matrix::translate(boundingSphere.center()));
    normal_point_->setMatrix(osg::Matrix::translate(boundingSphere.center()+osg::Vec3(0, -radius, 0)));
  }

  initilized_ = true;

  return;
}

void Registrator::clear()
{
  Renderable::clear();
  visualization_->setMatrix(osg::Matrix::identity());
  visualization_->removeChildren(0, visualization_->getNumChildren());
}

void Registrator::visualizeError(void)
{
  if (error_vertices_->empty())
    return;

  size_t partition_size = 10000;
  size_t item_num = error_vertices_->size()/2;
  size_t partition_num = (item_num+partition_size-1)/partition_size;
  osg::Geode* geode = new osg::Geode;
  for (size_t i = 0, i_end = partition_num; i < i_end; ++ i) {
//    osg::UIntArray* vertices_indices = OSGUtility::generateIndices(partition_size, i, item_num, 2);
//    osg::UIntArray* color_indices = OSGUtility::generateIndices(partition_size, i, item_num);

    osg::Geometry* geometry = new osg::Geometry;
    geometry->setUseDisplayList(true);
    geometry->setUseVertexBufferObjects(true);
//    geometry->setVertexData(osg::Geometry::ArrayData(error_vertices_, vertices_indices, osg::Geometry::BIND_PER_PRIMITIVE));
//    geometry->setColorData(osg::Geometry::ArrayData(error_colors_, color_indices, osg::Geometry::BIND_PER_PRIMITIVE));
//    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertices_indices->size()));

    geode->addDrawable(geometry);
  }

  addChild(geode);
  return;
}

void Registrator::visualizeAxis()
{
  osg::BoundingSphere boundingSphere = MainWindow::getInstance()->getOSGViewerWidget()->getBound();
  double radius = boundingSphere.radius();
  double diameter = radius*2;
  double cone_length = 10;
  double cylinder_thickness = 1;
  double cone_thickness = 2;

  osg::Vec3 pivot_point = pivot_point_->getMatrix().getTrans();
  osg::Vec3 normal_point = normal_point_->getMatrix().getTrans();
  osg::Vec3 normal = normal_point-pivot_point;
  normal.normalize();
                                                
  osg::ref_ptr<osg::LineSegment> cone(new osg::LineSegment(normal_point-normal*cone_length, normal_point));
  visualization_->addChild(OSGUtility::drawCone(*cone, cone_thickness, osg::Vec4(0.6, 0.6, 0.6, 1.0)));

  double disc_radius = std::sqrt(radius*radius-(pivot_point-boundingSphere.center()).length2())/3;
  osg::ref_ptr<osg::LineSegment> disc(new osg::LineSegment(pivot_point-normal*0.01, pivot_point+normal*0.01));
  visualization_->addChild(OSGUtility::drawCylinder(*disc, disc_radius, osg::Vec4(0.79, 1.0, 0.44, 0.8)));
  

  osg::Vec3 axis_y = pivot_dragger_->getMatrix()*osg::Vec3(0,1,0);
  osg::Matrix rotation(osg::Matrix::rotate(axis_y,-normal));

  float t_scale = radius/4;
  float r_scale = radius/6;
  osg::Matrix flip(osg::Matrix::rotate(osg::Vec3(0, 1, 0), osg::Vec3(0, -1, 0)));        
  pivot_dragger_->setMatrix(flip*osg::Matrix::scale(t_scale, t_scale, t_scale)*rotation*osg::Matrix::translate(pivot_point));
  normal_dragger_->setMatrix(osg::Matrix::scale(r_scale, r_scale, r_scale)*rotation*osg::Matrix::translate(pivot_point));

  osg::ref_ptr<osg::LineSegment> cylinder_0(new osg::LineSegment(normal_point-normal*diameter, pivot_point));
  visualization_->addChild(OSGUtility::drawCylinder(*cylinder_0, cylinder_thickness, osg::Vec4(0.6, 0.6, 0.6, 1.0)));
  osg::ref_ptr<osg::LineSegment> cylinder_1(new osg::LineSegment(pivot_point+normal*t_scale, normal_point-normal*cone_length));
  visualization_->addChild(OSGUtility::drawCylinder(*cylinder_1, cylinder_thickness, osg::Vec4(0.6, 0.6, 0.6, 1.0)));

  addChild(pivot_point_);
  addChild(normal_point_);
  addChild(visualization_);
  addChild(pivot_dragger_);
  addChild(normal_dragger_);

  return;
}

void Registrator::updateImpl()
{
  if (show_error_)
    visualizeError();

  if (show_axis_)
    visualizeAxis();

  return;
}

bool Registrator::load(const QString& filename)
{
  FILE *file = fopen(filename.toStdString().c_str(),"r");
  if (file == NULL)
    return false;

  double x, y, z;
  double nx, ny, nz;
  fscanf(file, "%lf %lf %lf", &x, &y, &z);
  fscanf(file, "%lf %lf %lf", &nx, &ny, &nz);
  fclose(file);

  setPivotPoint(osg::Vec3(x, y, z));
  setAxisNormal(osg::Vec3(nx, ny, nz));

  return true;
}

bool Registrator::load(void)
{
  const QString& workspace = MainWindow::getInstance()->getWorkspace();
  return load(workspace+"/axis.txt");
}

void Registrator::save(const QString& filename)
{
  FILE *file = fopen(filename.toStdString().c_str(),"w");
  if (file == NULL)
    return;

  osg::Vec3 pivot_point = getPivotPoint();
  osg::Vec3 axis_normal = getAxisNormal();

  fprintf(file, "%f %f %f\n", pivot_point.x(), pivot_point.y(), pivot_point.z());
  fprintf(file, "%f %f %f\n", axis_normal.x(), axis_normal.y(), axis_normal.z());
  fclose(file);

  return;
}

void Registrator::save(void)
{
  MainWindow* main_window = MainWindow::getInstance();
  QString filename = QFileDialog::getSaveFileName(main_window,
    "Save Registrator", main_window->getWorkspace(), "Registrator (*.txt)");
  if (filename.isEmpty())
    return;

  save(filename);

  return;
}


osg::Matrix Registrator::getRotationMatrix(double angle) const
{
  osg::Vec3 pivot_point = getPivotPoint();
  osg::Vec3 axis_normal = getAxisNormal();

  osg::Matrix matrix = osg::Matrix::identity();
  matrix = matrix*osg::Matrix::translate(-pivot_point);
  matrix = matrix*osg::Matrix::rotate(angle, axis_normal);
  matrix = matrix*osg::Matrix::translate(pivot_point);

  return matrix;
}

void Registrator::saveRegisteredPoints(int object)
{
  QMutexLocker locker(&mutex_);

  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  std::string folder = model->getPointsFolder(object);
  if (folder.empty())
    return;

  PointCloud registered_points;
  for (size_t i = 0; i < 12; ++ i)
  {
    osg::ref_ptr<PointCloud> point_cloud = model->getPointCloud(object, i);
    if (!point_cloud->isRegistered())
      continue;

    const osg::Matrix& matrix = point_cloud->getMatrix();
    for (size_t j = 0, j_end = point_cloud->size(); j < j_end; ++ j)
    {
    PCLRichPoint plant_point = point_cloud->at(j);

    osg::Vec3 point(plant_point.x, plant_point.y, plant_point.z);
    point = matrix.preMult(point);
    plant_point.x = point.x();
    plant_point.y = point.y();
    plant_point.z = point.z();

    osg::Vec3 normal(plant_point.normal_x, plant_point.normal_y, plant_point.normal_z);
    normal = matrix.preMult(normal);
    plant_point.normal_x = normal.x();
    plant_point.normal_y = normal.y();
    plant_point.normal_z = normal.z();

    registered_points.push_back(plant_point);
    }
  }

  std::string filename = folder+"/points.pcd";
  registered_points.save(filename);

   filename = folder+"/points.asc";
   FILE *file = fopen(filename.c_str(),"w");
   if (file == NULL)
     return;
   for (size_t i = 0, i_end = registered_points.size(); i < i_end; ++ i)
   {
     const PCLRichPoint& point = registered_points[i];
     fprintf(file, "%f %f %f %d %d %d\n", point.x, point.y, point.z, point.r, point.g, point.b);
   }
   fclose(file);

  model->updatePointCloud(object);

  return;
}

void Registrator::saveRegisteredPoints(void)
{
  int object;
  if (!ParameterManager::getInstance().getObjectParameter(object))
    return;

  saveRegisteredPoints(object);
}

void Registrator::refineAxis(int object)
{
  QMutexLocker locker(&mutex_);

  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  std::vector<osg::Matrix> matrices;
  std::vector<double>      angles;
  for (size_t i = 1; i < 12; ++ i)
  {
    osg::ref_ptr<PointCloud> point_cloud = model->getPointCloud(object, i);
    if(!point_cloud->isRegistered())
      continue;
    matrices.push_back(point_cloud->getMatrix());
    double angle = (i < 7)?(-i*M_PI/6.0):((12-i)*M_PI/6.0);
    angles.push_back(angle);
  }

  if (matrices.empty())
    return;

  // solve the normal of the axis: http://en.wikipedia.org/wiki/Rotation_matrix#Determining_the_axis
  // we can directly solve the null space of A for Ax=0, but gesvd of boost binding of lapack doesn't work
  // so we add a regularization that u+v+w=1, and solve it as least square of Ax=b
  namespace ublas = boost::numeric::ublas;
  ublas::matrix<double> A(3*matrices.size()+1, 3);
  ublas::vector<double> x = ublas::zero_vector<double>(3);
  ublas::vector<double> b = ublas::zero_vector<double>(3*matrices.size()+1);
  for (size_t i = 0, i_end = matrices.size(); i < i_end; ++ i)
    for (size_t j = 0; j < 3; ++ j)
      for (size_t k = 0; k < 3; ++ k)
        A(i*3+j, k) = matrices[i](k, j)-((j==k)?(1.0):(0.0));
  size_t idx = 3*matrices.size();
  A(idx, 0) = 1; A(idx, 1) = 1; A(idx, 2) = 1; b(idx) = 1;
  osg::Vec3 normal = getAxisNormal();
  math_solvers::least_squares(A, b, x);
  normal = osg::Vec3(x(0), x(1), x(2));
  normal.normalize();
  setAxisNormal(normal);

  // solve the point of the axis, use the same idea as that used to solve the axis
  // here Rp=p, where R is the 4x4 matrix, and p is the homogeneous coordinate of the axis point
  // and we convert it into Ax=b format
  for (size_t i = 0, i_end = matrices.size(); i < i_end; ++ i)
    for (size_t j = 0; j < 3; ++ j)
      b(i*3+j) = -matrices[i](3, j);
  osg::Vec3 pivot_point = getPivotPoint();
  A(idx, 0)=0; A(idx, 1)=1; A(idx, 2)=0; b(idx) = pivot_point.y();

  math_solvers::least_squares(A, b, x);
  pivot_point = osg::Vec3(x(0), x(1), x(2));
  setPivotPoint(pivot_point);

  save((model->getPointsFolder(object)+"/axis.txt").c_str());

  return;
}

void Registrator::refineAxis(void)
{
  int object;
  if (!ParameterManager::getInstance().getObjectParameter(object))
    return;

  refineAxis(object);
}

void Registrator::computeError(int object)
{
  error_vertices_->clear();
  error_colors_->clear();

  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  std::vector<bool> shown_flag(12, false);
  shown_flag[0] = true;
  for (size_t i = 1; i < 12; ++ i)
  {
    osg::ref_ptr<PointCloud> point_cloud = model->getPointCloud(object, i);
    shown_flag[i] = point_cloud->isShown();
    if (shown_flag[i])
      point_cloud->initRotation();
  }

  std::vector<std::pair<size_t, size_t> > neighbor_pairs;
  for (size_t i = 0; i < 11; ++ i)
    if (shown_flag[i] && shown_flag[i+1])
      neighbor_pairs.push_back(std::make_pair(i, i+1));
  if (shown_flag[0] && shown_flag[11])
    neighbor_pairs.push_back(std::make_pair(0, 11));

  PCLPointCloud::Ptr source(new PCLPointCloud);
  PCLPointCloud::Ptr target(new PCLPointCloud);
  for (size_t i = 0, i_end = neighbor_pairs.size(); i < i_end; ++ i)
  {
    model->getPointCloud(object, neighbor_pairs[i].first)->getTransformedPoints(*source);
    model->getPointCloud(object, neighbor_pairs[i].second)->getTransformedPoints(*target);

    pcl::registration::CorrespondenceEstimation<PCLPoint, PCLPoint, float> correspondence_estimation;
    correspondence_estimation.setInputSource(source);
    correspondence_estimation.setInputTarget(target);

    double distance_threshold = ParameterManager::getInstance().getRegistrationMaxDistance();
    pcl::CorrespondencesPtr correspondences (new pcl::Correspondences);
    correspondence_estimation.determineReciprocalCorrespondences(*correspondences, distance_threshold);

    for (size_t i = 0, i_end = correspondences->size(); i < i_end; ++ i)
    {
      const pcl::Correspondence& correspondence = correspondences->at(i);

      /*error_vertices_->push_back(source->at(correspondence.index_query).cast<osg::Vec3>());
      error_vertices_->push_back(target->at(correspondence.index_match).cast<osg::Vec3>());
      error_colors_->push_back(ColorMap::Instance().getColor(ColorMap::JET, correspondence.distance, 0, distance_threshold));*/
    }
    }

  return;
}

void Registrator::registrationICP(int max_iterations, double max_distance, int object)
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  std::vector<osg::ref_ptr<PointCloud> > point_clouds;
  for (size_t i = 1; i < 6; ++ i)
  {
    osg::ref_ptr<PointCloud> front_cloud = model->getPointCloud(object, i);
    if (front_cloud->isShown())
      point_clouds.push_back(front_cloud);
    osg::ref_ptr<PointCloud> back_cloud = model->getPointCloud(object, 12-i);
    if (back_cloud->isShown())
      point_clouds.push_back(back_cloud);
  }
  osg::ref_ptr<PointCloud> center_cloud = model->getPointCloud(object, 6);
  if (center_cloud->isShown())
    point_clouds.push_back(center_cloud);
  if (point_clouds.empty())
    return;

  for (size_t i = 0, i_end = point_clouds.size(); i < i_end; ++ i)
    point_clouds[i]->initRotation();

  PCLPointCloud::Ptr source(new PCLPointCloud);
  PCLPointCloud::Ptr target(new PCLPointCloud);

  pcl::IterativeClosestPoint<PCLPoint, PCLPoint> icp;
  icp.setUseReciprocalCorrespondences(true);
  // Set the max correspondence distance (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance(max_distance);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations(max_iterations);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon(0.000001);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon(64);

  model->getPointCloud(object, 0)->getTransformedPoints(*target);
  for (size_t i = 0, i_end = point_clouds.size(); i < i_end; ++ i)
  {
    point_clouds[i]->getTransformedPoints(*source);

    icp.setInputSource(source);
    icp.setInputTarget(target);
    PCLPointCloud transformed_source;
    icp.align(transformed_source);

    osg::Matrix result_matrix = PclMatrixCaster<osg::Matrix>(icp.getFinalTransformation());
    point_clouds[i]->setMatrix(point_clouds[i]->getMatrix()*result_matrix);

    *target += transformed_source;
  }

  if (show_error_)
  {
    QMutexLocker locker(&mutex_);
    computeError(object);
  }

  expire();

  return;
}

void Registrator::registrationICP(void)
{
  int max_iterations, object;
  double max_distance;
  if (!ParameterManager::getInstance().getRegistrationICPParameters(max_iterations, max_distance, object))
    return;

    QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
  connect(watcher, SIGNAL(finished()), watcher, SLOT(deleteLater()));

  QString running_message = QString("ICP registration for object %1 is running!").arg(object);
  QString finished_message = QString("ICP registration for object %1 finished!").arg(object);
  Messenger* messenger = new Messenger(running_message, finished_message, this);
  connect(watcher, SIGNAL(started()), messenger, SLOT(sendRunningMessage()));
  connect(watcher, SIGNAL(finished()), messenger, SLOT(sendFinishedMessage()));

  watcher->setFuture(QtConcurrent::run(this, &Registrator::registrationICP, max_iterations, max_distance, object));

  return;
}

void Registrator::registrationLUM(int segment_threshold, int max_iterations, double max_distance, int object)
{
  std::cout << "registrationLUM: object " << object << " running..." << std::endl;

  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  for (size_t view = 0; view < 12; ++ view)
  {
    osg::ref_ptr<PointCloud> point_cloud = model->getPointCloud(object, view);
    point_cloud->initRotation();
    point_cloud->setRegisterState(true);
  }

  int lum_max_iterations = 16;
  int outer_loop_num = std::max(1, max_iterations/lum_max_iterations);
  for (size_t loop = 0; loop < outer_loop_num; ++ loop)
  {
    pcl::registration::LUM<PCLPoint> lum;
    for (size_t i = 0; i < 12; ++ i)
    {
      osg::ref_ptr<PointCloud> point_cloud = model->getPointCloud(object, i);
      point_cloud->initRotation();

      PCLPointCloud::Ptr transformed_cloud(new PCLPointCloud);
      point_cloud->getTransformedPoints(*transformed_cloud);

      lum.addPointCloud(transformed_cloud);
    }

    for (size_t i = 0; i < 12; ++ i)
    {
      int source_idx = i;
      int target_idx = (i==11)?(0):(i+1);
      pcl::registration::CorrespondenceEstimation<PCLPoint, PCLPoint, float> correspondence_estimation;
      correspondence_estimation.setInputSource(lum.getPointCloud(source_idx));
      correspondence_estimation.setInputTarget(lum.getPointCloud(target_idx));

      pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
      correspondence_estimation.determineReciprocalCorrespondences (*correspondences, max_distance);
      lum.setCorrespondences(source_idx, target_idx, correspondences);
    }

    lum.setMaxIterations(lum_max_iterations);
    lum.compute();

    for (size_t i = 0; i < 12; ++ i)
    {
      Eigen::Affine3f transformation = lum.getTransformation(i);
      osg::Matrix osg_transformation = PclMatrixCaster<osg::Matrix>(Eigen::Matrix4f(transformation.data()));
      osg::ref_ptr<PointCloud> point_cloud = model->getPointCloud(object, i);
      point_cloud->setMatrix(point_cloud->getMatrix()*osg_transformation);
      point_cloud->setRegisterState(true);
    }
  }

  if (show_error_)
  {
    QMutexLocker locker(&mutex_);
    computeError(object);
  }

  saveRegisteredPoints(object);
  refineAxis(object);

  expire();

  return;
}

void Registrator::registrationLUM(void)
{
  int segment_threshold, max_iterations, object;
  double max_distance;
  if (!ParameterManager::getInstance().getRegistrationLUMParameters(segment_threshold, max_iterations, max_distance, object))
    return;

  QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
  connect(watcher, SIGNAL(finished()), watcher, SLOT(deleteLater()));

  QString running_message = QString("LUM registration for object %1 is running!").arg(object);
  QString finished_message = QString("LUM registration for object %1 finished!").arg(object);
  Messenger* messenger = new Messenger(running_message, finished_message, this);
  connect(watcher, SIGNAL(started()), messenger, SLOT(sendRunningMessage()));
  connect(watcher, SIGNAL(finished()), messenger, SLOT(sendFinishedMessage()));

  watcher->setFuture(QtConcurrent::run(this, &Registrator::registrationLUM, segment_threshold, max_iterations, max_distance, object));

  return;
}

bool Registrator::isAxisAccurate()
{
  bool flag = false;

  return flag;
}

void Registrator::automaticRegistration(void)
{

}

void Registrator::toggleRendering()
{
  if (!show_axis_ && !show_error_)
  {
    hidden_ = false;
    show_axis_ = true;
  }
  else if (show_axis_ && !show_error_)
  {
    show_axis_ = false;
    show_error_ = true;
  }
  else if (!show_axis_ && show_error_)
  {
    hidden_ = true;
    show_error_ = false;
  }

  expire();

  return;
}