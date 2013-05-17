#include "renderable.h"
#include "update_callback.h"


UpdateCallback::UpdateCallback(Renderable* renderable)
  :renderable_(renderable)
{
}


UpdateCallback::~UpdateCallback(void)
{
}


void UpdateCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
  renderable_->update();

  return;
}

