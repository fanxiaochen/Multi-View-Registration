#pragma once 
#ifndef TYPES_H
#define TYPES_H

#include <pcl/point_types.h>

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointXYZRGBNormal PCLRichPoint;
typedef pcl::PointCloud<PCLPoint>  PCLPointCloud;
typedef pcl::PointCloud<PCLRichPoint>  PCLRichPointCloud;

template <class Matrix>
class PclMatrixCaster {
public:
  PclMatrixCaster(const Eigen::Matrix4f& m)
    : m_(m)
  {}

  PclMatrixCaster(const Matrix& m)
  {
    for (int i = 0; i < 4; ++ i)
      for (int j = 0; j < 4; ++ j)
        m_(i, j) = m(j, i);
  }

  operator Eigen::Matrix4f() const
  {
    return m_;
  }

  operator Matrix() const
  {
    Matrix m;
    for (int i = 0; i < 4; ++ i)
      for (int j = 0; j < 4; ++ j)
        m(i, j) = m_(j, i);
    return m;
  }

private:
  Eigen::Matrix4f m_;
};

#endif

   