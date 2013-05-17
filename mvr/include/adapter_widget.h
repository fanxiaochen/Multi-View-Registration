#pragma once
#ifndef ADAPTER_WIDGET_H
#define ADAPTER_WIDGET_H

#include <QSet>
#include <QEvent>
#include <QMutex>
#include <QQueue>
#include <QGLWidget>
#include <osgViewer/GraphicsWindow>

class QInputEvent;
class ThreadedPainter;

class AdapterWidget : public QGLWidget
{

public:

  AdapterWidget( QWidget* parent = NULL, const QGLWidget* shareWidget = NULL, Qt::WindowFlags f = 0);
  virtual ~AdapterWidget();

  osgViewer::GraphicsWindow* getGraphicsWindow() { return _gw.get(); }

  void AdapterWidget::setKeyboardModifiers( QInputEvent* event );

  virtual void keyPressEvent( QKeyEvent* event );
  virtual void keyReleaseEvent( QKeyEvent* event );
  virtual void mousePressEvent( QMouseEvent* event );
  virtual void mouseReleaseEvent( QMouseEvent* event );
  virtual void mouseDoubleClickEvent( QMouseEvent* event );
  virtual void mouseMoveEvent( QMouseEvent* event );
  virtual void wheelEvent( QWheelEvent* event );

protected:

  osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _gw;          

  friend class ThreadedPainter;

  virtual void resizeEvent( QResizeEvent* event );
  virtual void moveEvent( QMoveEvent* event );
  virtual void glDraw();
};

#endif // ADAPTER_WIDGET_H