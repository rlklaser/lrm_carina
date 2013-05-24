/*
 *  Copyright (C) 2012, Laboratorio de Robotica Movel - ICMC/USP
 *  Rafael Luiz Klaser <rlklaser@gmail.com>
 *  http://lrm.icmc.usp.br
 *
 *  Apoio FAPESP: 2012/04555-4
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file ogreimp.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Feb 4, 2013
 *
 */

#include <ros/ros.h>

#include <OGRE/OgreMesh.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgrePass.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreTextureUnitState.h>
#include <OGRE/OgreMeshSerializer.h>
#include <OGRE/OgreSubMesh.h>
#include <OGRE/OgreHardwareBufferManager.h>

#if defined(IS_ASSIMP3)
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#else
#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>
#include <assimp/IOStream.h>
#include <assimp/IOSystem.h>
#endif

#include <resource_retriever/retriever.h>
#include <string>
#include <sstream>
#include <iostream>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

// Mostly stolen from gazebo
void buildMesh(const aiScene* scene, const aiNode* node, const Ogre::MeshPtr& mesh, Ogre::AxisAlignedBox& aabb, float& radius)
{
  if (!node)
  {
    return;
  }

  aiMatrix4x4 transform = node->mTransformation;
  aiNode *pnode = node->mParent;
  while (pnode)
  {
    // Don't convert to y-up orientation, which is what the root node in
    // Assimp does
    if (pnode->mParent != NULL)
      transform = pnode->mTransformation * transform;
    pnode = pnode->mParent;
  }

  aiMatrix3x3 rotation(transform);
  aiMatrix3x3 inverse_transpose_rotation(rotation);
  inverse_transpose_rotation.Inverse();
  inverse_transpose_rotation.Transpose();

  for (uint32_t i = 0; i < node->mNumMeshes; i++)
  {
    aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];

    Ogre::SubMesh* submesh = mesh->createSubMesh();
    submesh->useSharedVertices = false;
    submesh->vertexData = new Ogre::VertexData();
    Ogre::VertexData* vertex_data = submesh->vertexData;
    Ogre::VertexDeclaration* vertex_decl = vertex_data->vertexDeclaration;

    size_t offset = 0;
    // positions
    vertex_decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

    // normals
    if (input_mesh->HasNormals())
    {
      vertex_decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
      offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    }

    // texture coordinates (only support 1 for now)
    if (input_mesh->HasTextureCoords(0))
    {
      vertex_decl->addElement(0, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
      offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
    }

    // todo vertex colors

    // allocate the vertex buffer
    vertex_data->vertexCount = input_mesh->mNumVertices;
    Ogre::HardwareVertexBufferSharedPtr vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertex_decl->getVertexSize(0),
                                                                          vertex_data->vertexCount,
                                                                          Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
                                                                          false);

    vertex_data->vertexBufferBinding->setBinding(0, vbuf);
    float* vertices = static_cast<float*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

    // Add the vertices
    for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)
    {
      aiVector3D p = input_mesh->mVertices[j];
      p *= transform;
      *vertices++ = p.x;
      *vertices++ = p.y;
      *vertices++ = p.z;

      Ogre::Vector3 v(p.x, p.y, p.z);
      aabb.merge(v);
      float dist = v.length();
      if (dist > radius)
      {
        radius = dist;
      }

      if (input_mesh->HasNormals())
      {
	      aiVector3D n = inverse_transpose_rotation * input_mesh->mNormals[j];
        *vertices++ = n.x;
        *vertices++ = n.y;
        *vertices++ = n.z;
      }

      if (input_mesh->HasTextureCoords(0))
      {
        *vertices++ = input_mesh->mTextureCoords[0][j].x;
        *vertices++ = input_mesh->mTextureCoords[0][j].y;
      }
    }

    // calculate index count
    submesh->indexData->indexCount = 0;
    for (uint32_t j = 0; j < input_mesh->mNumFaces; j++)
    {
      aiFace& face = input_mesh->mFaces[j];
      submesh->indexData->indexCount += face.mNumIndices;
    }

    // If we have less than 65536 (2^16) vertices, we can use a 16-bit index buffer.
    if( vertex_data->vertexCount < (1<<16) )
    {
      // allocate index buffer
      submesh->indexData->indexBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
        Ogre::HardwareIndexBuffer::IT_16BIT,
        submesh->indexData->indexCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
        false);

      Ogre::HardwareIndexBufferSharedPtr ibuf = submesh->indexData->indexBuffer;
      uint16_t* indices = static_cast<uint16_t*>(ibuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

      // add the indices
      for (uint32_t j = 0; j < input_mesh->mNumFaces; j++)
      {
        aiFace& face = input_mesh->mFaces[j];
        for (uint32_t k = 0; k < face.mNumIndices; ++k)
        {
          *indices++ = face.mIndices[k];
        }
      }

      ibuf->unlock();
    }
    else
    {
      // Else we have more than 65536 (2^16) vertices, so we must
      // use a 32-bit index buffer (or subdivide the mesh, which
      // I'm too impatient to do right now)

      // allocate index buffer
      submesh->indexData->indexBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
        Ogre::HardwareIndexBuffer::IT_32BIT,
        submesh->indexData->indexCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
        false);

      Ogre::HardwareIndexBufferSharedPtr ibuf = submesh->indexData->indexBuffer;
      uint32_t* indices = static_cast<uint32_t*>(ibuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

      // add the indices
      for (uint32_t j = 0; j < input_mesh->mNumFaces; j++)
      {
        aiFace& face = input_mesh->mFaces[j];
        for (uint32_t k = 0; k < face.mNumIndices; ++k)
        {
          *indices++ = face.mIndices[k];
        }
      }

      ibuf->unlock();
    }
    vbuf->unlock();
  }

  for (uint32_t i=0; i < node->mNumChildren; ++i)
  {
    buildMesh(scene, node->mChildren[i], mesh, aabb, radius);
  }
}

void loadTexture(const std::string& resource_path)
{
  if (!Ogre::TextureManager::getSingleton().resourceExists(resource_path))
  {
    resource_retriever::Retriever retriever;
    resource_retriever::MemoryResource res;
    try
    {
      res = retriever.get(resource_path);
    }
    catch (resource_retriever::Exception& e)
    {
      ROS_ERROR("%s", e.what());
    }

    if (res.size != 0)
    {
      Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(res.data.get(), res.size));
      Ogre::Image image;
      std::string extension = fs::extension(fs::path(resource_path));

      if (extension[0] == '.')
      {
        extension = extension.substr(1, extension.size() - 1);
      }

      try
      {
        image.load(stream, extension);
        Ogre::TextureManager::getSingleton().loadImage(resource_path, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
      }
      catch (Ogre::Exception& e)
      {
        ROS_ERROR("Could not load texture [%s]: %s", resource_path.c_str(), e.what());
      }
    }
  }
}

// Mostly cribbed from gazebo
void loadMaterialsForMesh(const std::string& resource_path, const aiScene* scene, const Ogre::MeshPtr& mesh)
{
  std::vector<Ogre::MaterialPtr> material_lookup;

  for (uint32_t i = 0; i < scene->mNumMaterials; i++)
  {
    std::stringstream ss;
    ss << resource_path << "Material" << i;
    Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(ss.str(), ROS_PACKAGE_NAME, true);
    material_lookup.push_back(mat);

    Ogre::Technique* tech = mat->getTechnique(0);
    Ogre::Pass* pass = tech->getPass(0);

    aiMaterial *amat = scene->mMaterials[i];

    Ogre::ColourValue diffuse(1.0, 1.0, 1.0, 1.0);
    Ogre::ColourValue specular(1.0, 1.0, 1.0, 1.0);
    Ogre::ColourValue ambient(0.5, 0.5, 0.5, 1.0);

    for (uint32_t j=0; j < amat->mNumProperties; j++)
    {
      aiMaterialProperty *prop = amat->mProperties[j];
      std::string propKey = prop->mKey.data;

      if (propKey == "$tex.file")
      {
        aiString texName;
        aiTextureMapping mapping;
        uint32_t uvIndex;
        amat->GetTexture(aiTextureType_DIFFUSE,0, &texName, &mapping, &uvIndex);

        // Assume textures are in paths relative to the mesh
        std::string texture_path = fs::path(resource_path).parent_path().string() + "/" + texName.data;
        loadTexture(texture_path);
        Ogre::TextureUnitState* tu = pass->createTextureUnitState();
        tu->setTextureName(texture_path);
      }
      else if (propKey == "$clr.diffuse")
      {
        aiColor3D clr;
        amat->Get(AI_MATKEY_COLOR_DIFFUSE, clr);
        diffuse = Ogre::ColourValue(clr.r, clr.g, clr.b);
      }
      else if (propKey == "$clr.ambient")
      {
        aiColor3D clr;
        amat->Get(AI_MATKEY_COLOR_AMBIENT, clr);

        // Most of our DAE files don't have ambient color defined
        if (clr.r > 0 && clr.g > 0 && clr.b > 0)
        {
          ambient = Ogre::ColourValue(clr.r, clr.g, clr.b);
        }
      }
      else if (propKey == "$clr.specular")
      {
        aiColor3D clr;
        amat->Get(AI_MATKEY_COLOR_SPECULAR, clr);
        specular = Ogre::ColourValue(clr.r, clr.g, clr.b);
      }
      else if (propKey == "$clr.emissive")
      {
        aiColor3D clr;
        amat->Get(AI_MATKEY_COLOR_EMISSIVE, clr);
        mat->setSelfIllumination(clr.r, clr.g, clr.b);
      }
      else if (propKey == "$clr.opacity")
      {
        float o;
        amat->Get(AI_MATKEY_OPACITY, o);
        diffuse.a = o;
      }
      else if (propKey == "$mat.shininess")
      {
        float s;
        amat->Get(AI_MATKEY_SHININESS, s);
        mat->setShininess(s);
      }
      else if (propKey == "$mat.shadingm")
      {
        int model;
        amat->Get(AI_MATKEY_SHADING_MODEL, model);
        switch(model)
        {
          case aiShadingMode_Flat:
            mat->setShadingMode(Ogre::SO_FLAT);
            break;
          case aiShadingMode_Phong:
            mat->setShadingMode(Ogre::SO_PHONG);
            break;
          case aiShadingMode_Gouraud:
          default:
            mat->setShadingMode(Ogre::SO_GOURAUD);
            break;
        }
      }
    }

    int mode = aiBlendMode_Default;
    amat->Get(AI_MATKEY_BLEND_FUNC, mode);
    switch(mode)
    {
      case aiBlendMode_Additive:
        mat->setSceneBlending(Ogre::SBT_ADD);
        break;
      case aiBlendMode_Default:
      default:
        {
          if (diffuse.a < 0.99)
          {
            pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
          }
          else
          {
            pass->setSceneBlending(Ogre::SBT_REPLACE);
          }
        }
        break;
    }

    mat->setAmbient(ambient);
    mat->setDiffuse(diffuse);
    specular.a = diffuse.a;
    mat->setSpecular(specular);
  }

  for (uint32_t i = 0; i < mesh->getNumSubMeshes(); ++i)
  {
    mesh->getSubMesh(i)->setMaterialName(material_lookup[scene->mMeshes[i]->mMaterialIndex]->getName());
  }
}

Ogre::MeshPtr loadMeshFromResource(const std::string& resource_path)
{
  if (Ogre::MeshManager::getSingleton().resourceExists(resource_path))
  {
    return Ogre::MeshManager::getSingleton().getByName(resource_path);
  }
  else
  {
    fs::path model_path(resource_path);
#if BOOST_FILESYSTEM_VERSION == 3
    std::string ext = model_path.extension().string();
#else
    std::string ext = model_path.extension();
#endif
    if (ext == ".mesh" || ext == ".MESH")
    {
      resource_retriever::Retriever retriever;
      resource_retriever::MemoryResource res;
      try
      {
        res = retriever.get(resource_path);
      }
      catch (resource_retriever::Exception& e)
      {
        ROS_ERROR("%s", e.what());
        return Ogre::MeshPtr();
      }

      if (res.size == 0)
      {
        return Ogre::MeshPtr();
      }

      Ogre::MeshSerializer ser;
      Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(res.data.get(), res.size));
      Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(resource_path, "rviz");
      ser.importMesh(stream, mesh.get());

      return mesh;
    }
  }

  return Ogre::MeshPtr();
}

int main(int argc, char* argv[])
{
	return 0;
}
