#include <osg/Geode>
#include <osg/Point>
#include <osg/Depth>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Texture2D>
#include <osg/ShapeDrawable>
#include <osg/BlendFunc>
#include <osg/BlendColor>
#include <osg/LineSegment>

#include "osg_utility.h"

namespace OSGUtility
{
  osg::Geode* drawCylinder(const osg::LineSegment& axis, double thickness, const osg::Vec4& color)
  {
    return drawCylinder(axis.start(), axis.end(), thickness, color);
  }

  osg::Geode* drawCylinder(const osg::Vec3& top, const osg::Vec3& base, double thickness, const osg::Vec4& color)
  {
    osg::Geode* geode = new osg::Geode();

    osg::Vec3 center = (top+base)/2;
    osg::Vec3 offset = base-top;
    osg::Vec3 zAxis(0.0, 0.0, 1.0);
    osg::Vec3 rotation = zAxis^offset;
    float angle = acos((zAxis*offset)/offset.length());

    osg::Cylinder* cylinder = new osg::Cylinder(center, thickness, offset.length());
    cylinder->setRotation(osg::Quat(angle, rotation));

    osg::ShapeDrawable* drawable = new osg::ShapeDrawable(cylinder);
    drawable->setColor(color);
    geode->addDrawable(drawable);

    return geode;
  }

  osg::Geode* drawCone(const osg::LineSegment& axis, double thickness, const osg::Vec4& color)
  {
    return drawCone(axis.start(), axis.end(), thickness, color);
  }

  osg::Geode* drawCone(const osg::Vec3& top, const osg::Vec3& base, double radius, const osg::Vec4& color)
  {
    osg::Vec3 offset = base-top;
    osg::Vec3 zAxis(0.0, 0.0, 1.0);
    osg::Vec3 rotation = zAxis^offset;
    float angle = acos((zAxis*offset)/offset.length());
    osg::Cone* cone = new osg::Cone(top, radius, offset.length());
    cone->setRotation(osg::Quat(angle, rotation));

    osg::Geode* geode = new osg::Geode();
    osg::ShapeDrawable* drawable = new osg::ShapeDrawable(cone);
    drawable->setColor(color);
    geode->addDrawable(drawable);

    return geode;
  }

  osg::Geode* drawSphere(const osg::Vec3& center, double radius, const osg::Vec4& color)
  {
    osg::Geode* geode = new osg::Geode();

    osg::Sphere* sphere = new osg::Sphere(center, radius);
    osg::ShapeDrawable* drawable = new osg::ShapeDrawable(sphere);
    drawable->setColor(color);
    geode->addDrawable(drawable);

    return geode;
  }

  osg::Geode* drawBox(const osg::Vec3& center, double width, const osg::Vec4& color)
  {
    osg::Geode* geode = new osg::Geode();

    osg::Box* box = new osg::Box(center, width);
    osg::ShapeDrawable* drawable = new osg::ShapeDrawable(box);
    drawable->setColor(color);
    geode->addDrawable(drawable);

    return geode;
  }

  osg::Geode* drawTetrahedron(const osg::Vec3& center, double radius, const osg::Vec4& color)
  {
    double offset = 2*radius/std::sqrt(3);
    osg::Vec3 corner_1 = center + osg::Vec3(-offset, -offset, -offset);
    osg::Vec3 corner_2 = center + osg::Vec3(+offset, +offset, -offset);
    osg::Vec3 corner_3 = center + osg::Vec3(+offset, -offset, +offset);
    osg::Vec3 corner_4 = center + osg::Vec3(-offset, +offset, +offset);

    osg::ref_ptr<osg::Vec3Array>  vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array>  colors = new osg::Vec4Array;
    colors->push_back(color);

    vertices->push_back(corner_1); vertices->push_back(corner_2), vertices->push_back(corner_3);
    vertices->push_back(corner_1); vertices->push_back(corner_3), vertices->push_back(corner_4);
    vertices->push_back(corner_1); vertices->push_back(corner_2), vertices->push_back(corner_4);
    vertices->push_back(corner_2); vertices->push_back(corner_3), vertices->push_back(corner_4);

    osg::Geode* geode = new osg::Geode;
    osg::Geometry* geometry = new osg::Geometry;
    geometry->setUseDisplayList(true);
    geometry->setUseVertexBufferObjects(true);
    geometry->setVertexData(osg::Geometry::ArrayData(vertices, osg::Geometry::BIND_PER_PRIMITIVE));
    geometry->setColorData(osg::Geometry::ArrayData(colors, osg::Geometry::BIND_OVERALL));
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, vertices->size()));
    geode->addDrawable(geometry);

    return geode;
  }

  osg::Geode* drawPolygon(osg::Vec3Array* polygon, const osg::Vec4& color)
  {
    osg::Vec4Array* colors = new osg::Vec4Array;
    osg::Vec4 transparentColor = color;
    transparentColor.a() = 0.3f;
    colors->push_back(transparentColor);

    osg::Geometry* geometry = new osg::Geometry;
    geometry->setUseDisplayList(true);
    geometry->setUseVertexBufferObjects(true);
    geometry->setVertexArray(polygon);
    geometry->setColorArray(colors);
    geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON, 0, polygon->size()));

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable(geometry);

    geode->setName("polygon");

    osg::StateSet* stateset=geode->getOrCreateStateSet();

    stateset->setMode( GL_LIGHTING, osg::StateAttribute::OFF |
      osg::StateAttribute::PROTECTED);
    stateset->setMode(GL_BLEND,osg::StateAttribute::ON);
    stateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
    stateset->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );

    osg::BlendColor *bc=new osg::BlendColor(osg::Vec4(1.0,0,0,0));
    osg::BlendFunc *bf=new osg::BlendFunc();
    stateset->setAttributeAndModes(bf,osg::StateAttribute::ON);
    stateset->setAttributeAndModes(bc,osg::StateAttribute::ON);
    bf->setSource(osg::BlendFunc::CONSTANT_ALPHA);
    bf->setDestination(osg::BlendFunc::ONE_MINUS_CONSTANT_ALPHA);
    bc->setConstantColor(osg::Vec4(1, 1, 1,0.4f)); 

    return geode;
  }

  osg::Geode* drawBBox(osg::BoundingBox* bbox, const osg::Vec4& color)
  {
    osg::CompositeShape* composite_shape = new osg::CompositeShape();
    std::vector<osg::ref_ptr<osg::LineSegment> > edges;
    edges.push_back(new osg::LineSegment(osg::Vec3(bbox->xMin(), bbox->yMin(), bbox->zMin()), osg::Vec3(bbox->xMax(), bbox->yMin(), bbox->zMin())));
    edges.push_back(new osg::LineSegment(osg::Vec3(bbox->xMin(), bbox->yMin(), bbox->zMin()), osg::Vec3(bbox->xMin(), bbox->yMax(), bbox->zMin())));
    edges.push_back(new osg::LineSegment(osg::Vec3(bbox->xMin(), bbox->yMin(), bbox->zMin()), osg::Vec3(bbox->xMin(), bbox->yMin(), bbox->zMax())));

    edges.push_back(new osg::LineSegment(osg::Vec3(bbox->xMin(), bbox->yMax(), bbox->zMax()), osg::Vec3(bbox->xMax(), bbox->yMax(), bbox->zMax())));
    edges.push_back(new osg::LineSegment(osg::Vec3(bbox->xMin(), bbox->yMax(), bbox->zMax()), osg::Vec3(bbox->xMin(), bbox->yMin(), bbox->zMax())));
    edges.push_back(new osg::LineSegment(osg::Vec3(bbox->xMin(), bbox->yMax(), bbox->zMax()), osg::Vec3(bbox->xMin(), bbox->yMax(), bbox->zMin())));

    edges.push_back(new osg::LineSegment(osg::Vec3(bbox->xMax(), bbox->yMax(), bbox->zMin()), osg::Vec3(bbox->xMin(), bbox->yMax(), bbox->zMin())));
    edges.push_back(new osg::LineSegment(osg::Vec3(bbox->xMax(), bbox->yMax(), bbox->zMin()), osg::Vec3(bbox->xMax(), bbox->yMin(), bbox->zMin())));
    edges.push_back(new osg::LineSegment(osg::Vec3(bbox->xMax(), bbox->yMax(), bbox->zMin()), osg::Vec3(bbox->xMax(), bbox->yMax(), bbox->zMax())));

    edges.push_back(new osg::LineSegment(osg::Vec3(bbox->xMax(), bbox->yMin(), bbox->zMax()), osg::Vec3(bbox->xMin(), bbox->yMin(), bbox->zMax())));
    edges.push_back(new osg::LineSegment(osg::Vec3(bbox->xMax(), bbox->yMin(), bbox->zMax()), osg::Vec3(bbox->xMax(), bbox->yMax(), bbox->zMax())));
    edges.push_back(new osg::LineSegment(osg::Vec3(bbox->xMax(), bbox->yMin(), bbox->zMax()), osg::Vec3(bbox->xMax(), bbox->yMin(), bbox->zMin())));

    double radius = 0.01;
    for (size_t i = 0, i_end = edges.size(); i < i_end; ++ i) {
      osg::Vec3 center = (edges[i]->start()+edges[i]->end())/2;
      osg::Vec3 offset = edges[i]->end()-edges[i]->start();
      osg::Vec3 zAxis(0.0, 0.0, 1.0);
      osg::Vec3 rotation = zAxis^offset;
      float angle = acos((zAxis*offset)/offset.length());
      osg::Cylinder* cylinder = new osg::Cylinder(center, radius, offset.length());
      cylinder->setRotation(osg::Quat(angle, rotation));
      composite_shape->addChild(cylinder);
    }

    osg::Geode* geode = new osg::Geode();
    osg::ShapeDrawable* drawable = new osg::ShapeDrawable(composite_shape);
    drawable->setColor(color);
    geode->addDrawable(drawable);

    return geode;
  }

  void computeNodePathToRoot(osg::Node& node, osg::NodePath& np) {
    np.clear();
    osg::NodePathList nodePaths = node.getParentalNodePaths();
    if (!nodePaths.empty()) {
      np = nodePaths.front();
    }

    return;
  }

  osg::UIntArray* generateIndices(size_t partition_size, size_t partition_index, size_t item_num, size_t item_size)
  {
    size_t start = partition_size*partition_index;
    size_t end = partition_size*(partition_index+1);
    end = (end > item_num)?item_num:end;

    osg::UIntArray* indices = new osg::UIntArray();
    indices->reserve((end-start)*item_size);
    for (size_t i = start*item_size, i_end = end*item_size; i < i_end; ++ i)
      indices->push_back(i);

    return indices;
  }
};