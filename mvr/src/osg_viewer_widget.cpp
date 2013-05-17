#include <QFileDialog>
#include <QTextStream>
#include <QResizeEvent>

#include <osg/Depth>
#include <osg/Point>
#include <osg/LineWidth>
#include <osgDB/WriteFile>
#include <osg/AnimationPath>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>

#include "parameter.h"
#include "registrator.h"
#include "main_window.h"
#include "light_source.h"
#include "update_visitor.h"
#include "toggle_handler.h"
#include "parameter_dialog.h"
#include "osg_viewer_widget.h"


OSGViewerWidget::OSGViewerWidget(QWidget * parent, const QGLWidget * shareWidget, Qt::WindowFlags f)
  :AdapterWidget(parent, shareWidget, f),
  scene_root_(new osg::Group),
  other_root_(new osg::Group),
  gl_thread_(this),
  threaded_painter_(this)
{
  osg::ref_ptr<osg::Group> root = new osg::Group;
  root->addChild(scene_root_);
  root->addChild(other_root_);
  setSceneData(root);

  scene_root_->getOrCreateStateSet()->setAttribute(new osg::Point(2.0f), osg::StateAttribute::ON);
  scene_root_->getOrCreateStateSet()->setAttribute(new osg::LineWidth(1.0f), osg::StateAttribute::ON);

  setCameraManipulator(new osgGA::TrackballManipulator);
 
  addEventHandler(new osgViewer::HelpHandler);
  addEventHandler(new osgViewer::StatsHandler);
  addEventHandler(new osgViewer::LODScaleHandler);
  addEventHandler(new osgViewer::ThreadingHandler);

  for (int i = 0; i < 6; ++ i)
  {
    osg::ref_ptr<LightSource> light_source = new LightSource(i);
    light_sources_.push_back(light_source);
    char index = '0'+i;
    addEventHandler(new ToggleHandler(light_source, index, std::string("Toggle Light Source ")+index));
    addChild(light_source, false);
  }

  double w = width();
  double h = height();
  getCamera()->setViewport(new osg::Viewport(0, 0, w, h));
  getCamera()->setProjectionMatrixAsPerspective(60.0f, w/h, 1.0f, 10000.0f);
  getCamera()->setGraphicsContext(getGraphicsWindow());
  getCamera()->setClearColor(osg::Vec4(1, 1, 1, 1.0));

  setThreadingModel(osgViewer::Viewer::SingleThreaded);

  this->doneCurrent();
}

OSGViewerWidget::~OSGViewerWidget()
{
  osgViewer::View::EventHandlers handlers = getEventHandlers();
  for (osgViewer::View::EventHandlers::iterator it = handlers.begin(); it != handlers.end(); ++ it)
    this->removeEventHandler(*it);

  stopRendering();
}

void OSGViewerWidget::increaseLineWidth(void)
{
  osg::StateSet* state_Set = scene_root_->getOrCreateStateSet();
  osg::LineWidth* line_width = dynamic_cast<osg::LineWidth*>(state_Set->getAttribute(osg::StateAttribute::LINEWIDTH));
  if (line_width == NULL)
    return;

  if (line_width->getWidth() >= 16.0)
  {
    line_width->setWidth(16.0);
    return;
  }

  line_width->setWidth(line_width->getWidth()+1.0);
  return;
}

void OSGViewerWidget::decreaseLineWidth(void)
{
  osg::StateSet* state_Set = scene_root_->getOrCreateStateSet();
  osg::LineWidth* line_width = dynamic_cast<osg::LineWidth*>(state_Set->getAttribute(osg::StateAttribute::LINEWIDTH));
  if (line_width == NULL)
    return;

  if (line_width->getWidth() <= 1.0)
  {
    line_width->setWidth(1.0);
    return;
  }

  line_width->setWidth(line_width->getWidth()-1.0);
  return;
}

void OSGViewerWidget::increasePointSize(void)
{
  osg::StateSet* state_Set = scene_root_->getOrCreateStateSet();
  osg::Point* point = dynamic_cast<osg::Point*>(state_Set->getAttribute(osg::StateAttribute::POINT));
  if (point == NULL)
    return;

  if (point->getSize() >= 16.0)
  {
    point->setSize(16.0);
    return;
  }

  point->setSize(point->getSize()+1.0);
  return;
}

void OSGViewerWidget::decreasePointSize(void)
{
  osg::StateSet* state_Set = scene_root_->getOrCreateStateSet();
  osg::Point* point = dynamic_cast<osg::Point*>(state_Set->getAttribute(osg::StateAttribute::POINT));
  if (point == NULL)
    return;

  if (point->getSize() <= 1.0)
  {
    point->setSize(1.0);
    return;
  }

  point->setSize(point->getSize()-1.0);
  return;
}

void OSGViewerWidget::startRendering()
{
  addEventHandler(new osgGA::StateSetManipulator(getSceneData()->getOrCreateStateSet()));

  threaded_painter_.moveToThread(&gl_thread_);

  connect(&gl_thread_, SIGNAL(started()), &threaded_painter_, SLOT(start()));
  gl_thread_.start();
}

void OSGViewerWidget::stopRendering()
{
  threaded_painter_.stop();
  gl_thread_.wait();
}

void OSGViewerWidget::paintGL(void)
{
  QMutexLocker locker(&mutex_);
  frame();
}

void OSGViewerWidget::resizeEvent(QResizeEvent *event)
{
  threaded_painter_.resizeEvent(event);
}

void OSGViewerWidget::paintEvent(QPaintEvent * /*event*/)
{
  // Handled by the GLThread.
}

void OSGViewerWidget::closeEvent(QCloseEvent *event)
{
  stopRendering();
  QGLWidget::closeEvent(event);
}

void OSGViewerWidget::centerSceneIfNecessary(void)
{
  bool locked = mutex_.tryLock();
  scene_root_->accept(UpdateVisitor());
  if (locked)
    mutex_.unlock();

  osg::BoundingSphere bounding_sphere = getBound();
  if (bounding_sphere.center() == osg::Vec3(0, 0, 0))
    return;

  MainWindow::getInstance()->getRegistrator()->init();

  osg::Vec3d eye, center, up;
  getCamera()->getViewMatrixAsLookAt(eye, center, up);

  double c2 = (eye-bounding_sphere.center()).length2();
  osg::Vec3d view_direction = eye - center;
  view_direction.normalize();
  double a = (eye-bounding_sphere.center())*view_direction;
  double distance = std::sqrt(c2-a*a);
  if (distance < 16*bounding_sphere.radius()) // Line and sphere intersect
    return;

  centerSceneImpl();

  return;
}


void OSGViewerWidget::centerScene(void)
{
  QMutexLocker locker(&mutex_);
  scene_root_->accept(UpdateVisitor());

  centerSceneImpl();

  return;
}

void OSGViewerWidget::centerSceneImpl(void)
{
  osgGA::CameraManipulator* camera_manipulator = getCameraManipulator();

  osg::BoundingSphere bounding_sphere = getBound();
  double radius = bounding_sphere.radius();
  osg::Vec3d eye_offset(0.0, 0.0, -2*radius);
  camera_manipulator->setHomePosition(bounding_sphere.center() + eye_offset, bounding_sphere.center(), osg::Vec3d(0.0f,-1.0f,0.0f));
  camera_manipulator->home(0);

  double offset = radius/1.5;
  std::vector<osg::Vec3> offsets;
  offsets.push_back(osg::Vec3(0, 0, -offset));
  offsets.push_back(osg::Vec3(0, 0, offset));
  offsets.push_back(osg::Vec3(-offset, 0, 0));
  offsets.push_back(osg::Vec3(offset, 0, 0));
  offsets.push_back(osg::Vec3(0, -offset, 0));
  offsets.push_back(osg::Vec3(0, offset, 0));
  for (int i = 0; i < 6; ++ i)
    light_sources_[i]->init(bounding_sphere.center() + offsets[i]);

  return;
}

void OSGViewerWidget::replaceChild(osg::Node *old_child, osg::Node *new_child, bool in_scene)
{
  QMutexLocker locker(&mutex_);
  if (in_scene)
  {
    scene_root_->removeChild(old_child);
    scene_root_->addChild(new_child);
  }
  else
  {
    other_root_->removeChild(old_child);
    other_root_->addChild(new_child);
  }

  centerSceneIfNecessary();

  return;
}

void OSGViewerWidget::addChild(osg::Node *child, bool in_scene)
{
  QMutexLocker locker(&mutex_);
  if (in_scene)
    scene_root_->addChild(child);
  else
    other_root_->addChild(child);

  centerSceneIfNecessary();

  return;
}

void OSGViewerWidget::removeChild(osg::Node *child, bool in_scene)
{
  QMutexLocker locker(&mutex_);
  if (in_scene)
    scene_root_->removeChild(child);
  else
    other_root_->removeChild(child);

  return;
}

osg::BoundingSphere OSGViewerWidget::getBound(void) const
{
  return scene_root_->computeBound();
}

void OSGViewerWidget::removeChildren(bool in_scene)
{
  QMutexLocker locker(&mutex_);
  if (in_scene)
    scene_root_->removeChildren(0, scene_root_->getNumChildren());
  else
    other_root_->removeChildren(0, other_root_->getNumChildren());

  return;
}
