#ifndef __WAVEFRONTOBJMESH_HPP__
#define __WAVEFRONTOBJMESH_HPP__

#include <string>
#include <vector>
#include <unordered_map>
#include <Eigen/Eigen>
#include <stdint.h>
#include "tiny_obj_loader.h"
#include "Config.hpp"
#include "Reals.hpp"

PRJ_BEGIN
class WaveFrontObjMesh{
public:
  typedef struct Triangle {
    std::vector<unsigned int> findex;
    std::vector<Real3> vertices;
  }Triangle;
  typedef struct Key
  {
    Key(Real3 v)
    {
      vertex = v;
    }
    Real3 vertex;
    bool operator==(const Key &other) const
    {
      return vertex.isApprox(other.vertex);
    }
  }Key;
  typedef struct KeyHasher
  {
    std::size_t operator()(const Key& k) const
    {
      return ((std::hash<Real>()(k.vertex.x()) ^
              (std::hash<Real>()(k.vertex.y()) << 1)) >> 1) ^
              (std::hash<Real>()(k.vertex.z()) << 1);
    }
  }KeyHasher;  
public:
  WaveFrontObjMesh() {};
  ~WaveFrontObjMesh() {};

public:
  void loadForRender(const char *filename, const char *mtl_basepath = NULL);
  void loadForPhysics(const char *filename, const Real area = 0);
  void debug() const;
  size_t getNumShapes() const;
  size_t getVertexSize(size_t i = 0) const;
  size_t getNormalSize(size_t i = 0) const;
  size_t getIndexSize(size_t i = 0) const;
  size_t getNumTriangles(size_t i = 0) const;  
  Real3 getPoint(size_t i, size_t pidx) const;
  Real3 getFaceNormal(size_t i, size_t fidx) const;
  Triangle getTriangle(size_t i, size_t fidx) const;


  
  void *vertexData(size_t i = 0);
  void *normalData(size_t i = 0);
  void *indexData(size_t i = 0);
protected:
  /** \brief Calculate normals for each vertex.
   */
  void calculate_normal(size_t idx);

  void subdivision(size_t idx);

  void divideTriangel(Real3 v1, Real3 v2, Real3 v3, Real area,
                      std::unordered_map<Key, unsigned, KeyHasher> &vmap,
                      std::vector<unsigned int> &newIndices);
      
protected:
  std::string m_filename;
  std::vector<tinyobj::shape_t> m_shapes;
  Real _maxArea; ///< max area of triangle
};

PRJ_END

#endif // __WAVEFRONTOBJMESH_HPP__
