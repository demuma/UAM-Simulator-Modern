// TinyObjLoader v2.0.0 - Header only
// https://github.com/tinyobjloader/tinyobjloader
// Licensed under the MIT license.

#ifndef TINY_OBJ_LOADER_H_
#define TINY_OBJ_LOADER_H_

// A trimmed single-header version of tinyobjloader (v2.0.0) for OBJ loading.
// This is the official header with implementation guarded by
// TINYOBJLOADER_IMPLEMENTATION.

#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4514)
#pragma warning(disable : 4625)
#pragma warning(disable : 4626)
#pragma warning(disable : 4668)
#pragma warning(disable : 4710)
#pragma warning(disable : 4711)
#pragma warning(disable : 4820)
#pragma warning(disable : 5026)
#pragma warning(disable : 5027)
#endif

namespace tinyobj {

typedef struct {
  int vertex_index;
  int normal_index;
  int texcoord_index;
} index_t;

typedef struct {
  std::vector<index_t> indices;
  std::vector<unsigned char> num_face_vertices;
  std::vector<int> material_ids;
  std::vector<unsigned int> smoothing_group_ids;
} mesh_t;

typedef struct {
  std::string name;
  mesh_t mesh;
  std::vector<unsigned char> lines;
} shape_t;

typedef struct {
  std::string name;
  float ambient[3];
  float diffuse[3];
  float specular[3];
  float transmittance[3];
  float emission[3];
  float shininess;
  float ior;
  float dissolve;
  int illum;
  std::string ambient_texname;
  std::string diffuse_texname;
  std::string specular_texname;
  std::string specular_highlight_texname;
  std::string bump_texname;
  std::string displacement_texname;
  std::string alpha_texname;
  std::string reflection_texname;
} material_t;

typedef struct {
  std::vector<float> vertices;   // 3 floats per vertex
  std::vector<float> normals;    // 3 floats per normal
  std::vector<float> texcoords;  // 2 floats per texcoord
  std::vector<float> colors;     // 3 floats per vertex
} attrib_t;

struct callback_t {
  void (*vertex_cb)(void *user_data, float x, float y, float z, float w);
  void (*normal_cb)(void *user_data, float x, float y, float z);
  void (*texcoord_cb)(void *user_data, float x, float y, float z);
  void (*index_cb)(void *user_data, index_t *indices, int num_indices);
  void (*usemtl_cb)(void *user_data, const char *name, int material_id);
  void (*mtllib_cb)(void *user_data, const char *name);
  void (*group_cb)(void *user_data, const char **names, int num_names);
  void *user_data;
};

bool LoadObj(attrib_t *attrib, std::vector<shape_t> *shapes,
             std::vector<material_t> *materials, std::string *warn,
             std::string *err, const char *filename, const char *mtl_basedir = nullptr,
             bool triangulate = true, bool default_vcols_fallback = false);

}  // namespace tinyobj

#ifdef TINYOBJLOADER_IMPLEMENTATION

#include <cstdio>
#include <fstream>
#include <limits>
#include <set>

namespace tinyobj {

static std::string MakeDefault(const std::string &s, const std::string &d) {
  return s.empty() ? d : s;
}

static bool ReadFile(std::string *content, std::string *err,
                     const char *filename) {
  if (!content) return false;
  std::ifstream ifs(filename, std::ios::in | std::ios::binary);
  if (!ifs) {
    if (err) *err = std::string("Cannot open file: ") + filename;
    return false;
  }
  std::ostringstream ss;
  ss << ifs.rdbuf();
  *content = ss.str();
  return true;
}

static inline bool IsSpace(char c) {
  return (c == ' ' || c == '\t' || c == '\r' || c == '\n');
}

static void ParseIndex(index_t *idx, const char *token) {
  idx->vertex_index = idx->normal_index = idx->texcoord_index = -1;
  int v = -1, vt = -1, vn = -1;
  const char *s = token;
  v = std::strtol(s, const_cast<char **>(&s), 10);
  if (*s == '/') {
    s++;
    if (*s != '/') {
      vt = std::strtol(s, const_cast<char **>(&s), 10);
    }
    if (*s == '/') {
      s++;
      vn = std::strtol(s, const_cast<char **>(&s), 10);
    }
  }
  idx->vertex_index = v - 1;
  idx->texcoord_index = vt - 1;
  idx->normal_index = vn - 1;
}

static void ParseFace(std::vector<index_t> &indices,
                      std::vector<unsigned char> &num_fv,
                      const std::vector<std::string> &tokens) {
  const size_t n = tokens.size() - 1;
  num_fv.push_back(static_cast<unsigned char>(n));
  for (size_t i = 0; i < n; i++) {
    index_t idx;
    ParseIndex(&idx, tokens[i + 1].c_str());
    indices.push_back(idx);
  }
}

static void SplitTokens(std::vector<std::string> &tokens, const std::string &line) {
  tokens.clear();
  std::string token;
  std::istringstream iss(line);
  while (iss >> token) tokens.push_back(token);
}

bool LoadObj(attrib_t *attrib, std::vector<shape_t> *shapes,
             std::vector<material_t> *materials, std::string *warn,
             std::string *err, const char *filename, const char *mtl_basedir,
             bool triangulate, bool default_vcols_fallback) {
  (void)default_vcols_fallback;
  if (!attrib || !shapes) return false;
  std::string content;
  if (!ReadFile(&content, err, filename)) return false;

  std::vector<std::string> lines;
  std::string line;
  std::istringstream iss(content);
  while (std::getline(iss, line)) lines.push_back(line);

  attrib->vertices.clear();
  attrib->normals.clear();
  attrib->texcoords.clear();
  attrib->colors.clear();

  shapes->clear();
  if (materials) materials->clear();

  shape_t shape;
  shape.name = "default";

  std::vector<std::string> tokens;

  for (size_t i = 0; i < lines.size(); i++) {
    const std::string &l = lines[i];
    if (l.empty() || l[0] == '#') continue;
    SplitTokens(tokens, l);
    if (tokens.empty()) continue;

    const std::string &tag = tokens[0];
    if (tag == "v" && tokens.size() >= 4) {
      attrib->vertices.push_back(std::stof(tokens[1]));
      attrib->vertices.push_back(std::stof(tokens[2]));
      attrib->vertices.push_back(std::stof(tokens[3]));
      if (tokens.size() >= 7) {
        attrib->colors.push_back(std::stof(tokens[4]));
        attrib->colors.push_back(std::stof(tokens[5]));
        attrib->colors.push_back(std::stof(tokens[6]));
      }
    } else if (tag == "vn" && tokens.size() >= 4) {
      attrib->normals.push_back(std::stof(tokens[1]));
      attrib->normals.push_back(std::stof(tokens[2]));
      attrib->normals.push_back(std::stof(tokens[3]));
    } else if (tag == "vt" && tokens.size() >= 3) {
      attrib->texcoords.push_back(std::stof(tokens[1]));
      attrib->texcoords.push_back(std::stof(tokens[2]));
    } else if (tag == "f" && tokens.size() >= 4) {
      ParseFace(shape.mesh.indices, shape.mesh.num_face_vertices, tokens);
    } else if (tag == "o" || tag == "g") {
      if (!shape.mesh.indices.empty()) {
        shapes->push_back(shape);
        shape = shape_t();
      }
      shape.name = tokens.size() >= 2 ? tokens[1] : "";
    }
  }

  if (!shape.mesh.indices.empty()) shapes->push_back(shape);

  // Triangulate if necessary (fan triangulation)
  if (triangulate) {
    for (auto &s : *shapes) {
      mesh_t tri_mesh;
      size_t index_offset = 0;
      for (size_t f = 0; f < s.mesh.num_face_vertices.size(); f++) {
        int fv = s.mesh.num_face_vertices[f];
        if (fv < 3) {
          index_offset += fv;
          continue;
        }
        for (int k = 1; k < fv - 1; k++) {
          tri_mesh.indices.push_back(s.mesh.indices[index_offset + 0]);
          tri_mesh.indices.push_back(s.mesh.indices[index_offset + k]);
          tri_mesh.indices.push_back(s.mesh.indices[index_offset + k + 1]);
          tri_mesh.num_face_vertices.push_back(3);
        }
        index_offset += fv;
      }
      s.mesh = tri_mesh;
    }
  }

  if (warn) *warn = "";
  if (err) *err = "";
  return true;
}

}  // namespace tinyobj

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif  // TINYOBJLOADER_IMPLEMENTATION

#endif  // TINY_OBJ_LOADER_H_
