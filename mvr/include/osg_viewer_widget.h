#pragma once
#ifndef OSG_VIEWER_WIDGET_H
#define OSG_VIEWER_WIDGET_H

#include <QMutex>
#include <QThread>
#include <QGLWidget>
#include <osgViewer/Viewer>
#include <osg/BoundingSphere>

#include "threaded_painter.h"
#include "adapter_widget.h"

class LightSource;
class QResizeEvent;          

namespace osg
{
  class Node;
}

class OSGViewerWidget : public AdapterWidget, public osgViewer::Viewer
{
  Q_OBJECT
public:
  OSGViewerWidget(QWidget * parent = 0, const QGLWidget * shareWidget = 0, Qt::WindowFlags f = 0);
  virtual ~OSGViewerWidget();

  void startRendering();
  void stopRendering();

  void centerSceneIfNecessary(void);

  void replaceChild(osg::Node *old_child, osg::Node *new_child, bool in_scene);
  void addChild(osg::Node *child, bool in_scene=true);
  void removeChild(osg::Node *child, bool in_scene=true);
  void removeChildren(bool in_scene=true);
  osg::BoundingSphere getBound(void) const;

  inline osg::Group* getSceneRoot(void) {return scene_root_.get();}
  inline std::vector<osg::ref_ptr<LightSource> >& getLightSources(void) {return light_sources_;}

  public slots:
    void increasePointSize(void);
    void decreasePointSize(void);
    void increaseLineWidth(void);
    void decreaseLineWidth(void);
    void centerScene(void);

protected:
  void centerSceneImpl(void);
  virtual void paintGL(void);
  virtual void resizeEvent(QResizeEvent *event);
  virtual void paintEvent(QPaintEvent *event);
  virtual void closeEvent(QCloseEvent *event);

  osg::ref_ptr<osg::Group>                scene_root_;
  osg::ref_ptr<osg::Group>                other_root_;
  QThread                                 gl_thread_;
  ThreadedPainter                         threaded_painter_;
  std::vector<osg::ref_ptr<LightSource> > light_sources_;
  QMutex                                  mutex_;

};


#endif // OSG_VIEWER_WIDGET_H