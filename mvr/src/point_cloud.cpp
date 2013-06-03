#include <QRegExp>
#include <QFileInfo>
#include <QFileDialog>
#include <QColorDialog>
#include <QFutureWatcher>
#include <QtConcurrentRun>

#include <osg/Geode>
#include <osg/io_utils>
#include <osg/Geometry>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/TranslateAxisDragger>

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>


#include "parameter.h"
#include "registrator.h"
#include "main_window.h"
#include "parameter_dialog.h"
#include "file_system_model.h"
#include "parameter_manager.h"
#include "point_cloud.h"
#include "osg_viewer_widget.h"


PointCloud::PointCloud(void)
  :translate_dragger_(new osgManipulator::TranslateAxisDragger),
  trackball_dragger_(new osgManipulator::TrackballDragger),
  show_draggers_(false),
  registered_(false),
  triangulation_(new CGAL::Delaunay()),
  point_graph_(new boost::PointGraph()),
  point_graph_threshold_(-1.0),
  points_num_(0),
  noise_points_num_(0)
{
  translate_dragger_->setupDefaultGeometry();
  translate_dragger_->setHandleEvents(true);
  translate_dragger_->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
  translate_dragger_->setActivationKeyEvent('d');

  trackball_dragger_->setupDefaultGeometry();
  trackball_dragger_->setHandleEvents(true);
  trackball_dragger_->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
  trackball_dragger_->setActivationKeyEvent('d');

  translate_dragger_->addTransformUpdating(this);
  translate_dragger_->addTransformUpdating(trackball_dragger_);

  trackball_dragger_->addTransformUpdating(this);
  trackball_dragger_->addTransformUpdating(translate_dragger_);
}

PointCloud::~PointCloud(void)
{
  delete triangulation_;
  delete point_graph_;
}

void PointCloud::setRegisterState(bool registered)
{
  registered_=registered;
  if (registered_ && getView() != 0)
    saveTransformation();

  return;
}

bool PointCloud::open(const std::string& filename)
{
  clearData();

  QMutexLocker locker(&mutex_);

  if (pcl::io::loadPCDFile(filename, *this) != 0)
    return false;

  filename_ = filename;
  loadTransformation();

  registered_ = (getView() == 0) || (!(getMatrix().isIdentity()));
 
  expire();

  return true;
}

bool PointCloud::save(const std::string& filename)
{
  if (QString(filename.c_str()).right(3) == "ply")
  {
    PCLPointCloud point_cloud;
    osg::Vec3 pivot_point(-13.382786, 50.223461, 917.477600);
    osg::Vec3 axis_normal(-0.054323, -0.814921, -0.577020);
    osg::Matrix transformation = osg::Matrix::translate(-pivot_point)*osg::Matrix::rotate(axis_normal, osg::Vec3(0, 0, 1));
    for (size_t i = 0; i < points_num_; ++ i)
    {
      osg::Vec3 point(at(i).x, at(i).y, at(i).z);
      point = transformation.preMult(point);
      point_cloud.push_back(PCLPoint(point.x(), point.y(), point.z()));     
    }
    pcl::PLYWriter ply_writer;
    if (ply_writer.write<PCLPoint>(filename, point_cloud) != 0)
      return false;
  }
  else
  {
    pcl::PCDWriter pcd_writer;
    if (pcd_writer.writeBinaryCompressed<PCLRichPoint>(filename, *this) != 0)       
    return false;
  }

  return true;
}

void PointCloud::save(void)
{
  MainWindow* main_window = MainWindow::getInstance();
  QString filename = QFileDialog::getSaveFileName(main_window, "Save Point Cloud",
    main_window->getWorkspace(), "Point Cloud (*.pcd *.ply)");
  if (filename.isEmpty())
    return;

  save(filename.toStdString());

  return;
}

void PointCloud::reload(void)
{
  clearData();
  open(filename_);

  return;
}

void PointCloud::clearData()
{
  QMutexLocker locker(&mutex_);

  Renderable::clear();
  PCLRichPointCloud::clear();

  return;
}

void PointCloud::visualizePoints(size_t start, size_t end)
{
  osg::ref_ptr<osg::Vec3Array>  vertices = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec3Array>  normals_vertices = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec3Array>  orientations_vertices = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec3Array>  normals = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec4Array>  colors = new osg::Vec4Array;

  for (size_t i=start, i_end = end; i < i_end; i++)
  {
    const PCLRichPoint& point = at(i);
    vertices->push_back(osg::Vec3(point.x, point.y, point.z));
    normals->push_back(osg::Vec3(point.normal_x, point.normal_y, point.normal_z));
    colors->push_back(osg::Vec4(point.r/255.0, point.g/255.0, point.b/255.0, 1.0));
  }

  size_t item_num = vertices->size();

  osg::Geode* geode = new osg::Geode;
 
  osg::Geometry* geometry = new osg::Geometry;
  geometry->setUseDisplayList(true);
  geometry->setUseVertexBufferObjects(true);
  geometry->setVertexData(osg::Geometry::ArrayData(vertices, osg::Geometry::BIND_PER_VERTEX));
  geometry->setNormalData(osg::Geometry::ArrayData(normals, osg::Geometry::BIND_PER_VERTEX));
  geometry->setColorData(osg::Geometry::ArrayData(colors, osg::Geometry::BIND_PER_VERTEX));
  geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, item_num));

  geode->addDrawable(geometry);
   
  
  addChild(geode);

  return;
}

void PointCloud::updateImpl()
{

  visualizePoints(0, size());

  if (show_draggers_)
  {
    osg::BoundingSphere boundingSphere = getBound();
    osg::Matrix trans = osg::Matrix::translate(boundingSphere.center());
    double radius = boundingSphere.radius();
    float t_scale = radius/4;
    float r_scale = radius/8;
    osg::Matrix flip(osg::Matrix::rotate(osg::Vec3(0, 1, 0), osg::Vec3(0, -1, 0)));
    translate_dragger_->setMatrix(flip*osg::Matrix::scale(t_scale, t_scale, t_scale)*trans);
    trackball_dragger_->setMatrix(osg::Matrix::scale(r_scale, r_scale, r_scale)*trans);

    addChild(translate_dragger_);
    addChild(trackball_dragger_);
  }

  return;
}



PointCloud* PointCloud::getPrevObject(void)
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  return model->getPointCloud(getObject()-1);
}

PointCloud* PointCloud::getNextObject(void)
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  return model->getPointCloud(getObject()+1);
}



int PointCloud::getObject(void) const      
{
  QRegExp object("[\\/]object_([0-9]{5,5})[\\/]");
  object.indexIn(filename_.c_str());
  QString index = object.cap(1);

  return index.toInt();
}

int PointCloud::getView(void) const
{
  QRegExp object("[\\/]view_([0-9]{2,2})[\\/]");
  object.indexIn(filename_.c_str());
  QString index = object.cap(1);
  if (index.isEmpty())
    return 12;

  return index.toInt();
}

void PointCloud::setRotation(void)
{
  ParameterDialog parameter_dialog("Rotation Parameters", MainWindow::getInstance());
  int view = getView();
  DoubleParameter rotation_angle("Rotation Angle", "Rotation Angle", (view<7)?(-view*30):(12-view)*30, -180, 180, 30);
  parameter_dialog.addParameter(&rotation_angle);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return;

  double angle = rotation_angle*M_PI/180.0;
  setMatrix(MainWindow::getInstance()->getRegistrator()->getRotationMatrix(angle));

  if (angle == 0.0)
    deleteTransformation();

  return;
}

void PointCloud::toggleDraggers(void)
{
  QMutexLocker locker(&mutex_);
  show_draggers_ = !show_draggers_;
  expire();

  return;
}

void PointCloud::toggleRegisterState(void)
{
  registered_ = !registered_;
  if (registered_)
    saveTransformation();   
  else
    deleteTransformation();

  return;
}


void PointCloud::getTransformedPoints(PCLPointCloud& points)
{
  points.clear();

  const osg::Matrix& matrix = getMatrix();
  for (size_t i = 0, i_end = size(); i < i_end; ++ i)
  {
    osg::Vec3 point(at(i).x, at(i).y, at(i).z);
    point = matrix.preMult(point);
    points.push_back(PCLPoint(point.x(), point.y(), point.z()));
  }

  return;
}

void PointCloud::loadTransformation(void)
{
  std::string filename = (QFileInfo(filename_.c_str()).path()+"/transformation.txt").toStdString();
  FILE *file = fopen(filename.c_str(),"r");
  if (file == NULL)
    return;

  osg::Matrix matrix;
  for (int i = 0; i < 4; ++ i)
  {
    for (int j = 0; j < 4; ++ j)
    {
      double element;
      fscanf(file, "%lf", &element);
      matrix(j, i) = element;
    }
  }
  setMatrix(matrix);
  fclose(file);

  return;
}

void PointCloud::saveTransformation(void)
{
  std::string filename = (QFileInfo(filename_.c_str()).path()+"/transformation.txt").toStdString();
  FILE *file = fopen(filename.c_str(),"w");
  if (file == NULL)
    return;

  osg::Matrix matrix = getMatrix();
  for (int i = 0; i < 4; ++ i)
  {
    for (int j = 0; j < 4; ++ j)
    {
      fprintf(file, "%lf ", matrix(j, i));
    }
    fprintf(file, "\n");
  }
  fclose(file);

  return;
}

void PointCloud::deleteTransformation(void)
{
  std::string filename = (QFileInfo(filename_.c_str()).path()+"/transformation.txt").toStdString();
  std::remove(filename.c_str());
}


void PointCloud::registration(int segment_threshold, int max_iterations, double max_distance)
{
  if (getView() != 12)
    return;

  Registrator* registrator = MainWindow::getInstance()->getRegistrator();
  registrator->registrationLUM(segment_threshold, max_iterations, max_distance, getObject());

  expire();

  return;
}

void PointCloud::registration(void)
{
  int segment_threshold, max_iterations;
  double max_distance;
  if (!ParameterManager::getInstance().getRegistrationLUMParameters(segment_threshold, max_iterations, max_distance))
    return;

  QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
  connect(watcher, SIGNAL(finished()), watcher, SLOT(deleteLater()));

  int object = getObject();
  QString running_message = QString("Computing registration for object %1!").arg(object);
  QString finished_message = QString("Registration for object %1 computed!").arg(object);
  Messenger* messenger = new Messenger(running_message, finished_message, this);
  connect(watcher, SIGNAL(started()), messenger, SLOT(sendRunningMessage()));
  connect(watcher, SIGNAL(finished()), messenger, SLOT(sendFinishedMessage()));

  watcher->setFuture(QtConcurrent::run(this, &PointCloud::registration, segment_threshold, max_iterations, max_distance));

  return;
}


bool PointCloud::isShown(void) const
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  return model->isShown(filename_);
  return false;
}

void PointCloud::initRotation(void)
{
  if (!getMatrix().isIdentity())
    return;

  int view = getView();
  if (view == 0)
    return;

  double angle = ((view<7)?(-view):(12-view))*M_PI/6;
  setMatrix(MainWindow::getInstance()->getRegistrator()->getRotationMatrix(angle));

  return;
}

//struct CompareByLabel
//{
//  bool operator()(const PCLRichPoint& a, const PCLRichPoint& b)
//  {
//    return a.label > b.label;
//  }
//};

void PointCloud::denoise(int segment_threshold, double triangle_length)
{
  QMutexLocker locker(&mutex_);
  
  points_num_ = size();
  std::cout<<"triangle length:"<<ParameterManager::getInstance().getTriangleLength()<<std::endl;
  initPointGraph(ParameterManager::getInstance().getTriangleLength());

  boost::PointGraph& g_point = *point_graph_;
  std::cout<<"g_point size:"<<boost::num_vertices(g_point)<<std::endl;
  std::vector<boost::PointGraphTraits::vertex_descriptor> component(boost::num_vertices(g_point));
  size_t component_num = boost::connected_components(g_point, &component[0]);
  std::cout<<"component num:"<<component_num<<std::endl;

  std::vector<std::vector<boost::PointGraphTraits::vertex_descriptor> > components(component_num);
  for (size_t i = 0; i < points_num_; ++ i)
    components[component[i]].push_back(i);
  
  osg::ref_ptr<PointCloud> denoised_cloud = new PointCloud; 
  for (size_t i = 0, i_end = components.size(); i < i_end; ++ i)
  {
    std::cout<<components[i].size()<<std::endl;
    if (components[i].size() >= segment_threshold)
    {
      for (size_t j = 0, j_end = components[i].size(); j < j_end; ++ j)
        denoised_cloud->push_back(at(components[i][j]));
      std::cout<<"denoised_cloud size:"<<denoised_cloud->size()<<std::endl;
      continue;
    }

    noise_points_num_ += components[i].size();
  }

  for(size_t i = 0, i_end = denoised_cloud->size(); i < i_end; i++)
    at(i) = denoised_cloud->at(i);
  points_num_ = denoised_cloud->size();
  int k = points_num_;
  for(std::vector<PCLRichPoint,Eigen::aligned_allocator<PCLRichPoint>>::iterator it 
    = points.begin()+points_num_; it != points.end(); it++)
  {
    points.erase(it);
    std::cout<<"remove "<<k--<<std::endl;
  }
    

  triangulation_->clear();
  g_point.clear();

  expire();

  return;
}

void PointCloud::initPointGraph(double distance_threshold)
{
  if (distance_threshold == point_graph_threshold_
    && boost::num_edges(*point_graph_) != 0)
    return;

  triangulate();
  point_graph_threshold_ = distance_threshold;

  boost::PointGraph& g_point = *point_graph_;
  g_point = boost::PointGraph(points_num_);

  size_t t = 8;

  for (CGAL::Delaunay::Finite_edges_iterator it = triangulation_->finite_edges_begin();
    it != triangulation_->finite_edges_end(); ++ it)
  {
    const CGAL::Delaunay::Cell_handle& cell_handle  = it->first;
    const CGAL::Delaunay::Vertex_handle& source_handle = cell_handle->vertex(it->second);
    const CGAL::Delaunay::Vertex_handle& target_handle = cell_handle->vertex(it->third);
    double distance_L1 = std::sqrt(CGAL::squared_distance(source_handle->point(), target_handle->point()));
    if (distance_L1 > point_graph_threshold_)
      continue;

    size_t source_id = source_handle->info();
    size_t target_id = target_handle->info();
    assert (source_id < plant_points_num_ && target_id < plant_points_num_);
    WeightedEdge weighted_edge(distance_L1);
    boost::add_edge(source_id, target_id, weighted_edge, g_point);
  }

  return;
}

void PointCloud::triangulate() const
{
  if (triangulation_->number_of_vertices() != 0)
    return;
  std::cout<<"initial triangulation vertices num:"<<triangulation_->number_of_vertices()<<std::endl;
  for (size_t i = 0, i_end = points_num_; i < i_end; ++ i)
  {
    const PCLRichPoint& point = at(i);
    CGAL::Delaunay::Point delaunay_point(point.x, point.y, point.z);
    CGAL::Delaunay::Vertex_handle vertex_handle = triangulation_->insert(delaunay_point);
    vertex_handle->info() = i;
  }
  std::cout<<"final triangulation vertices num:"<<triangulation_->number_of_vertices()<<std::endl;
  return;
}