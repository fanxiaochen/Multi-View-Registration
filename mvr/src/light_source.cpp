#include <osg/LineSegment>
#include <osg/LightSource>
#include <osgManipulator/TranslateAxisDragger>

#include "main_window.h"
#include "osg_viewer_widget.h"
#include "light_source.h"


LightSource::LightSource(unsigned int num)
  :light_source_(new osg::LightSource()),
  dragger_(new osgManipulator::TranslateAxisDragger),
  initialized_(false)
{
  toggleRendering();
  osg::ref_ptr<osg::Light> light = new osg::Light;
  light->setLightNum(num);
  light->setPosition(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
  light_source_->setLight(light);

  dragger_->setupDefaultGeometry();
  dragger_->setHandleEvents(true);
  dragger_->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
  dragger_->setActivationKeyEvent('d');

  dragger_->addTransformUpdating(this);
}


LightSource::~LightSource(void)
{
}

void LightSource::init(const osg::Vec3& position, const osg::Vec4& ambient,
                       const osg::Vec4& diffuse, const osg::Vec4& specular)
{
  if (initialized_)
    return;

  light_source_->getLight()->setPosition(osg::Vec4(position.x(), position.y(), position.z(), 1.0f));
  light_source_->getLight()->setAmbient(ambient);
  light_source_->getLight()->setDiffuse(diffuse);
  light_source_->getLight()->setSpecular(specular);

  addChild(light_source_.get());

  initialized_ = true;

  return;
}

osg::Light* LightSource::getLight(void)
{
  return light_source_->getLight();
}

void LightSource::clear()
{
  Renderable::clear();
  addChild(light_source_.get());
}

void LightSource::updateImpl()
{
  osg::BoundingSphere boundingSphere = MainWindow::getInstance()->getOSGViewerWidget()->getBound();
  double radius = boundingSphere.radius();
  float t_scale = radius/6;
  osg::Matrix flip(osg::Matrix::rotate(osg::Vec3(0, 1, 0), osg::Vec3(0, -1, 0)));
  const osg::Vec4& pos4 = light_source_->getLight()->getPosition();
  osg::Vec3 position(pos4.x(), pos4.y(), pos4.z());
  if (pos4.w() != 0.0f)
    position = position/pos4.w();
  dragger_->setMatrix(flip*osg::Matrix::scale(t_scale, t_scale, t_scale)*osg::Matrix::translate(position));
  addChild(dragger_);

  return;
}