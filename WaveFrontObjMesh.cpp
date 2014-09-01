#include <algorithm>
#include <Eigen/Dense>
#include "WaveFrontObjMesh.hpp"
USE_PRJ_NAMESPACE;

void WaveFrontObjMesh::loadForRender(const char *filename, const char *mtl_basepath)
{
  m_filename = std::string(filename);
  mtl_basepath = NULL;
  std::string err = tinyobj::LoadObj(m_shapes, filename, mtl_basepath);
  INFO("Load obj info: %s", err.c_str());
  ASSERT_MSG(err.empty(), "%s", err.c_str());
  for ( size_t i = 0; i < m_shapes.size(); i++ )
  {
    if ( m_shapes[i].mesh.normals.empty() )
      calculate_normal(i);
  }
  INFO("4");
}

void WaveFrontObjMesh::loadForPhysics(const char *filename, const Real area)
{
  m_filename = std::string(filename);  
  _maxArea = area;

  std::string err = tinyobj::LoadObj(m_shapes, filename);
  ASSERT_MSG(err.empty(), "%s", err.c_str());
  if(_maxArea > 0)
  {
    for ( size_t i = 0; i < m_shapes.size(); i++ ) 
      subdivision(i);
  }
}

void WaveFrontObjMesh::debug() const
{
  for ( size_t i=0; i < m_shapes.size(); i++ ) {
    const tinyobj::shape_t & shape = m_shapes[i];
    const std::vector<unsigned int> & indices = shape.mesh.indices;
    const std::vector<float> & positions = shape.mesh.positions;
    const std::vector<float> & normals = shape.mesh.normals;

    INFO("Shape %lu: %s", i, shape.name.c_str());
    ASSERT(indices.size() % 3 == 0);
#if 0
    for ( size_t j=0; j < indices.size(); j+=3 ) {
      printf("triangle: (%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f)  (%.2f, %.2f, %.2f)\n",
          positions[3*indices[j  ]], positions[3*indices[j  ]+1], positions[3*indices[j  ]+2],
          positions[3*indices[j+1]], positions[3*indices[j+1]+1], positions[3*indices[j+1]+2],
          positions[3*indices[j+2]], positions[3*indices[j+2]+1], positions[3*indices[j+2]+2]);

      if ( !normals.empty() ) {
        printf("normals: (%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f)  (%.2f, %.2f, %.2f)\n",
            normals[3*indices[j  ]], normals[3*indices[j  ]+1], normals[3*indices[j  ]+2],
            normals[3*indices[j+1]], normals[3*indices[j+1]+1], normals[3*indices[j+1]+2],
            normals[3*indices[j+2]], normals[3*indices[j+2]+1], normals[3*indices[j+2]+2]);
      }
    }
#endif
    INFO("number of triangles = %lu", indices.size() / 3);
    INFO("number of vertices = %lu", positions.size() / 3);
    INFO("indices.size() = %lu", indices.size());
    INFO("positions.size() = %lu", positions.size());
    INFO("normals.size() = %lu", normals.size());
    INFO("min of index = %u", *(std::min_element(indices.begin(), indices.end())));
    INFO("max of index = %u", *(std::max_element(indices.begin(), indices.end())));
  }
}

size_t WaveFrontObjMesh::getNumShapes() const
{
  return m_shapes.size();
}

size_t WaveFrontObjMesh::getVertexSize(size_t i) const
{
  ASSERT(m_shapes[i].mesh.positions.size() % 3 == 0);
  return m_shapes[i].mesh.positions.size() / 3;
}

size_t WaveFrontObjMesh::getNormalSize(size_t i) const
{
  return m_shapes[i].mesh.normals.size() / 3;
}

size_t WaveFrontObjMesh::getIndexSize(size_t i) const
{
  return m_shapes[i].mesh.indices.size();
}

Real3 WaveFrontObjMesh::getPoint(size_t i, size_t pidx) const
{
  const std::vector<float> & positions = m_shapes[i].mesh.positions;
  return Real3(positions[3*pidx+0],
               positions[3*pidx+1],
               positions[3*pidx+2]);
}

size_t WaveFrontObjMesh::getNumTriangles(size_t i) const
{
  ASSERT(m_shapes[i].mesh.indices.size() % 3 == 0);
  return m_shapes[i].mesh.indices.size() / 3;
}

Real3 WaveFrontObjMesh::getFaceNormal(size_t i, size_t fidx) const
{
  const std::vector<float> & fnormals = m_shapes[i].mesh.fnormals;
  return Real3(fnormals[3*fidx+0],
                 fnormals[3*fidx+1],
                 fnormals[3*fidx+2]);
}

WaveFrontObjMesh::Triangle WaveFrontObjMesh::getTriangle(size_t i, size_t fidx) const
{
  const std::vector<unsigned int> & indices = m_shapes[i].mesh.indices;
  const std::vector<float> & positions = m_shapes[i].mesh.positions;

  Triangle tri;
  tri.findex.push_back(indices[3*fidx+0]);
  tri.findex.push_back(indices[3*fidx+1]);  
  tri.findex.push_back(indices[3*fidx+2]);
  
  tri.vertices.push_back(Real3(positions[3*tri.findex[0]+0],
                               positions[3*tri.findex[0]+1],
                               positions[3*tri.findex[0]+2]));

  tri.vertices.push_back(Real3(positions[3*tri.findex[1]+0],
                               positions[3*tri.findex[1]+1],
                               positions[3*tri.findex[1]+2]));

  tri.vertices.push_back(Real3(positions[3*tri.findex[2]+0],
                               positions[3*tri.findex[2]+1],
                               positions[3*tri.findex[2]+2]));

  return tri;
}

void *WaveFrontObjMesh::vertexData(size_t i)
{
  return (void*)&(m_shapes[i].mesh.positions[0]);
}

void *WaveFrontObjMesh::normalData(size_t i)
{
  return (void*)&(m_shapes[i].mesh.normals[0]);
}

void *WaveFrontObjMesh::indexData(size_t i)
{
  return (void*)&(m_shapes[i].mesh.indices[0]);
}

void WaveFrontObjMesh::calculate_normal(size_t idx)
{
  // Index is assumed
  ASSERT(!m_shapes[idx].mesh.indices.empty());

  const std::vector<unsigned int> & indices = m_shapes[idx].mesh.indices;
  const std::vector<float> & positions = m_shapes[idx].mesh.positions;
  std::vector<float> & fnormals = m_shapes[idx].mesh.fnormals;
  fnormals.resize(indices.size());
  // To store normal for each vertex
  std::vector<Real3> new_normals(positions.size()/3, Real3::Zero());

  // For each face, calculate face normal, add it to each vertex of the face
  for ( size_t i=0; i < indices.size(); i += 3 )
  {
    Real3 a(positions[3*indices[i+1]  ]-positions[3*indices[i]  ],
            positions[3*indices[i+1]+1]-positions[3*indices[i]+1],
            positions[3*indices[i+1]+2]-positions[3*indices[i]+2]);
    Real3 b(positions[3*indices[i+2]  ]-positions[3*indices[i]  ],
            positions[3*indices[i+2]+1]-positions[3*indices[i]+1],
            positions[3*indices[i+2]+2]-positions[3*indices[i]+2]);
    Real3 n = a.cross(b).normalized();

    fnormals[i+0] = n.x();
    fnormals[i+1] = n.y();
    fnormals[i+2] = n.z();    
        
    new_normals[indices[i  ]] += n;
    new_normals[indices[i+1]] += n;
    new_normals[indices[i+2]] += n;
  }
  for ( size_t i=0; i < new_normals.size(); i++ )
  {
    new_normals[i].normalize();
  }

  std::vector<float> & normals = m_shapes[idx].mesh.normals;
  if ( !normals.empty() )
    WARN("Overwriting exisiting normals...");
  normals.resize(positions.size());

  for ( size_t i=0; i < indices.size(); i++ )
  {
    normals[3*indices[i]  ] = new_normals[indices[i]].x();
    normals[3*indices[i]+1] = new_normals[indices[i]].y();
    normals[3*indices[i]+2] = new_normals[indices[i]].z();
  }
}

void WaveFrontObjMesh::subdivision(size_t idx)
{
  // Index is assumed
  ASSERT(!m_shapes[idx].mesh.indices.empty());

  //std::vector<float> newPositions;
  std::vector<unsigned int> newIndices;  
  std::unordered_map<Key, unsigned, KeyHasher> vmap;

  for(size_t f = 0; f < getNumTriangles(); ++f)
  {
    WaveFrontObjMesh::Triangle tri = getTriangle(0, f);
    Real3 a = tri.vertices[1] - tri.vertices[0];
    Real3 b = tri.vertices[2] - tri.vertices[0];
    Real area = (a.cross(b)).norm() * 0.5;
    divideTriangel(tri.vertices[0],tri.vertices[1],tri.vertices[2],area,vmap,newIndices);
  }

  // std::swap(m_shapes[idx].mesh.positions, newPositions);
  std::vector<float> &positions = m_shapes[idx].mesh.positions;
  positions.resize(vmap.size() * 3);

  std::unordered_map<Key, unsigned, KeyHasher>::iterator ite = vmap.begin();

  for(; ite != vmap.end(); ++ite)
  {
    unsigned int i = ite->second;
    positions[3*i+0] = ite->first.vertex.x();
    positions[3*i+1] = ite->first.vertex.y();
    positions[3*i+2] = ite->first.vertex.z();    
  }
  std::swap(m_shapes[idx].mesh.indices, newIndices);
}

void WaveFrontObjMesh::divideTriangel(Real3 v1, Real3 v2, Real3 v3, Real area,
                                      std::unordered_map<Key, unsigned, KeyHasher> &vmap,
                                      std::vector<unsigned int> &newIndices)
{
  static unsigned idx = 0;

  if (area > _maxArea)
  {
    Real3 v12 = (v1 + v2) * 0.5;
    Real3 v13 = (v1 + v3) * 0.5;
    Real3 v23 = (v2 + v3) * 0.5;

    divideTriangel(v1,v12,v13, 0.25 * area, vmap, newIndices);
    divideTriangel(v2,v23,v12, 0.25 * area, vmap, newIndices);
    divideTriangel(v3,v13,v23, 0.25 * area, vmap, newIndices);
    divideTriangel(v13,v12,v23, 0.25 * area, vmap, newIndices);
  }
  else
  {
    Key keyValue1(v1);

    std::unordered_map<Key,unsigned,KeyHasher>::iterator ite = vmap.find(keyValue1);

    if(ite != vmap.end())
      newIndices.push_back(ite->second);

    else
    {
      vmap.insert(std::unordered_map<Key,unsigned,KeyHasher>::value_type(keyValue1,idx));
      newIndices.push_back(idx);
      ++idx;
    }

    Key keyValue2(v2);

    ite = vmap.find(keyValue2);

    if(ite != vmap.end())
      newIndices.push_back(ite->second);

    else
    {
      vmap.insert(std::unordered_map<Key,unsigned,KeyHasher>::value_type(keyValue2,idx));
      newIndices.push_back(idx);
      ++idx;
    }

    Key keyValue3(v3);

    ite = vmap.find(keyValue3);

    if(ite != vmap.end())
      newIndices.push_back(ite->second);

    else
    {
      vmap.insert(std::unordered_map<Key,unsigned,KeyHasher>::value_type(keyValue3,idx));
      newIndices.push_back(idx);
      ++idx;
    }
  }
}
