#pragma once
#ifndef LIGHT_SOURCE_H
#define LIGHT_SOURCE_H

#include "renderable.h"

namespace osg
{
  class Light;
  class LightSource;
  class TranslateAxisDragger;
}

namespace osgManipulator
{
  class TranslateAxisDragger;
}

class LightSource : public Renderable
{
public:
  LightSource(unsigned int num);
  virtual ~LightSource(void);

  virtual const char* className() const {return "LightSource";}

  void init(const osg::Vec3& position, const osg::Vec4& ambient=osg::Vec4(1, 1, 1, 1),
    const osg::Vec4& diffuse=osg::Vec4(1, 1, 1, 1), const osg::Vec4& specular=osg::Vec4(1, 1, 1, 1));

  osg::Light* getLight(void);

protected:
  virtual void clear();
  virtual void updateImpl();

private:
  osg::ref_ptr<osg::LightSource>                      light_source_;
  osg::ref_ptr<osgManipulator::TranslateAxisDragger>  dragger_;
  bool                                                initialized_;
};

#endif // LIGHT_SOURCE_H