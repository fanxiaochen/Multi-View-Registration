#include <sstream>
#include <algorithm>

#include <osg/Geode>
#include <osg/StateSet>
#include <osg/Material>
#include <osg/NodeCallback>

#include "update_callback.h"

#include "renderable.h"

Renderable::Renderable(void)
  :osg::MatrixTransform(),
  expired_(true),
  hidden_(false)
{
  setUpdateCallback(new UpdateCallback(this));
  this->setDataVariance(osg::Object::DYNAMIC);
}

Renderable::~Renderable(void)
{
}

void Renderable::expire()
{
  expired_ = true;

  return;
}

void Renderable::update()
{
  if (!expired_)
    return;

  if (!mutex_.tryLock())
    return;

  expired_ = false;

  clear();

  if (hidden_)
  {
    mutex_.unlock();
    return;
  }

  updateImpl();

  mutex_.unlock();
  return;
}

void Renderable::clear()
{
  removeChildren(0, getNumChildren());
  return;
}

void Renderable::toggleRendering()
{
  hidden_ = !hidden_;

  expire();

  return;
}