#pragma once
#ifndef UPDATE_VISITOR_H
#define UPDATE_VISITOR_H

#include <osg/NodeVisitor>

class UpdateVisitor : public osg::NodeVisitor
{
public:
  UpdateVisitor(void);
  ~UpdateVisitor(void);

  virtual void apply( osg::Node& node );
};

#endif // UPDATE_VISITOR_H