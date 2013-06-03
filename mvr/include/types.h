#pragma once 
#ifndef TYPES_H
#define TYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//struct RichPoint: public pcl::PointXYZRGBNormal
//{
//public:
//  enum LABEL{LABEL_NOISE, LABEL_OBJECT} label;
//};

typedef pcl::PointXYZ PCLPoint;
//typedef RichPoint PCLRichPoint;
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

//struct WeightedEdge
//{
//  WeightedEdge():length(0){}
//  WeightedEdge(double len):length(len){}
//  double length;
//};
//
//namespace boost
//{
//  struct setS;
//  struct vecS;
//  struct bidirectionalS;
//  struct undirectedS;
//  struct no_property;
//  enum edge_weight_t;
//  struct listS;
//
//  template <class OutEdgeListS, // a Sequence or an AssociativeContainer
//  class VertexListS, // a Sequence or a RandomAccessContainer
//  class DirectedS,
//  class VertexProperty = no_property,
//  class EdgeProperty = no_property,
//  class GraphProperty = no_property,
//  class EdgeListS = listS>
//  class adjacency_list;
//
//  template <typename G>
//  struct graph_traits;
//
//  typedef adjacency_list<setS, vecS, undirectedS, no_property, WeightedEdge> PointGraph;
//  typedef graph_traits<PointGraph> PointGraphTraits;
//}
//
//namespace CGAL
//{
//  class Epick;
//  typedef Epick Exact_predicates_inexact_constructions_kernel;
//  typedef Exact_predicates_inexact_constructions_kernel   K;
//
//  template < typename TDS = void >
//  class Triangulation_ds_vertex_base_3;
//
//  template < typename GT, typename DSVb = Triangulation_ds_vertex_base_3<> >
//  class Triangulation_vertex_base_3;
//
//  template < typename TDS = void >
//  class Triangulation_ds_cell_base_3;
//
//  template < typename Info_, typename GT, typename Vb = Triangulation_vertex_base_3<GT> >
//  class Triangulation_vertex_base_with_info_3;
//  typedef Triangulation_vertex_base_with_info_3<size_t, K>  Vb;
//
//  template < class Vb = Triangulation_ds_vertex_base_3<>, class Cb = Triangulation_ds_cell_base_3<> >
//  class Triangulation_data_structure_3;
//  typedef Triangulation_data_structure_3<Vb>                Tds;
//
//  struct Default;
//  template < class Gt, class Tds_, class Location_policy >
//  class Delaunay_triangulation_3;
//  typedef Delaunay_triangulation_3<K, Tds, Default>                  Delaunay;
//}

#endif

   