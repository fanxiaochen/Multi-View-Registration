#include <osg/Geode>

#include "renderable.h"
#include "update_visitor.h"

UpdateVisitor::UpdateVisitor()
  : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
{
}

UpdateVisitor::~UpdateVisitor(void)
{
}


void UpdateVisitor::apply( osg::Node& node )
{
  Renderable* renderable = dynamic_cast<Renderable*>(&node);
  if (renderable != NULL) {
    renderable->update();
  }

  traverse( node );

  return;
}