#pragma once
#ifndef UpdateRenderGeometryCallback_H
#define UpdateRenderGeometryCallback_H

#include <osg/NodeCallback>

namespace osg {
  class Node;
  class NodeVisitor;
}

class Renderable;

class UpdateCallback : public osg::NodeCallback
{
public:
  UpdateCallback(Renderable* renderable);
  virtual ~UpdateCallback(void);

  virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);

private:
  Renderable*   renderable_;
};

#endif // UpdateRenderGeometryCallback_H