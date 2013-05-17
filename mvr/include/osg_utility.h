#pragma once
#ifndef OSG_UTILITY_H
#define OSG_UTILITY_H

#include <osg/Node>
#include <osg/Array>
#include <osg/ref_ptr>
#include <osgViewer/View>
#include <osg/BoundingBox>
#include <osgViewer/ViewerEventHandlers>

namespace osg{
  class Geode;
  class LineSegment;
}

namespace OSGUtility
{
  osg::Geode* drawCylinder(const osg::LineSegment& axis, double thickness, const osg::Vec4& color);

  osg::Geode* drawCylinder(const osg::Vec3& top, const osg::Vec3& base, double thickness, const osg::Vec4& color);

  osg::Geode* drawCone(const osg::LineSegment& axis, double radius, const osg::Vec4& color);

  osg::Geode* drawCone(const osg::Vec3& top, const osg::Vec3& base, double radius, const osg::Vec4& color);

  osg::Geode* drawVertex(const osg::Vec3& center, double thickness, const osg::Vec4& color);

  osg::Geode* drawSphere(const osg::Vec3& center, double radius, const osg::Vec4& color);

  osg::Geode* drawBox(const osg::Vec3& center, double width, const osg::Vec4& color);

  osg::Geode* drawTetrahedron(const osg::Vec3& center, double radius, const osg::Vec4& color);

  osg::Geode* drawPolygon(osg::Vec3Array* polygon, const osg::Vec4& color);

  osg::Geode* drawBBox(osg::BoundingBox* bbox, const osg::Vec4& color);

  osg::UIntArray* generateIndices(size_t partition_size, size_t partition_index, size_t item_num, size_t item_size=1);

  void computeNodePathToRoot(osg::Node& node, osg::NodePath& np);

  template <class T>
  T* computeIntersection(osgViewer::View* view,
    const osgGA::GUIEventAdapter& ea,
    osgUtil::LineSegmentIntersector::Intersection& intersection,
    osg::NodePath& nodePath)
  {
    T* intersectionObject = NULL;

    osgUtil::LineSegmentIntersector::Intersections intersections;

    float x = ea.getX();
    float y = ea.getY();

    if (!view->computeIntersections(x,y,intersections)) {
      return intersectionObject;
    }

    if (intersections.size() == 0) {
      return intersectionObject;
    }

    intersection = *intersections.begin();
    nodePath = intersection.nodePath;

    while (!nodePath.empty()) {
      intersectionObject = dynamic_cast<T*>(nodePath.back());
      if (intersectionObject != NULL) {
        break;
      }
      nodePath.pop_back();
    }

    return intersectionObject;
  }
};

#endif // OSG_UTILITY_H
