#include <string>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <algorithm>
#include <stack>
#include <cstring>

//#define GLM_FORCE_SWIZZLE
#include "glm/glm.hpp"
#include "glm/gtx/intersect.hpp"

#define USE_DOUBLE     (1)

#if !(USE_DOUBLE)
typedef float  Float;
typedef glm::vec3 Vec3;
#else
typedef double Float;
typedef glm::f64vec3 Vec3;
#endif

namespace {
  static const int TET_FACES[4][3] = {
    {2, 1, 0}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3},
  };
  static const Vec3 DIRS[] = {
    {(Float)+1.0, (Float) 0.0, (Float) 0.0}, // right
    {(Float) 0.0, (Float)-1.0, (Float) 0.0}, // down
    {(Float) 0.0, (Float)+1.0, (Float) 0.0}, // up
    {(Float)-1.0, (Float)-1.0, (Float) 0.0}, // left
    {(Float) 0.0, (Float) 0.0, (Float) 1.0}, // forward
    {(Float) 0.0, (Float) 0.0, (Float)-1.0}, // back
  };
};

struct Tetrahedron {
  int id0;
  int id1;
  int id2;
  int id3;
};

struct Triangle {
  int id0;
  int id1;
  int id2;
};

struct Edge {
  int e[4];
  Edge(int e0, int e1, int e2, int e3) : e{e0, e1, e2, e3} {}
};

struct Context {
  std::string              fn_srf_tri; // surface.obj(triangle)
  std::string              fn_vol_tet; // volume.obj(tetrahedron)
  std::vector<Vec3>        tri_vertices;
  std::vector<Triangle>    triangles;
  std::vector<Vec3>        tet_vertices;
  std::vector<Vec3>        out_vertices;
  std::vector<Tetrahedron> tetrahedron_faces;
  std::vector<Triangle>    triangle_faces;
  int                      resolution;
  int                      min_quality_exp;
  bool                     one_face_per_tet;
  Float                    tet_scale;
  Context() : fn_srf_tri(), fn_vol_tet(), tri_vertices(), triangles(), tet_vertices(), out_vertices(), tetrahedron_faces(), triangle_faces(), resolution(10), min_quality_exp(-3), one_face_per_tet(true), tet_scale((Float)0.8) {}
};

struct Tree {
  Context* ctx;
  Float ray_cast(Vec3& origin, Vec3& direction, Vec3* normal) { // use BVH for optimize
    glm::vec3 point(0.0f, 0.0f, 0.0f);
/*
    if (glm::intersectLineTriangle(origin, direction, v0, v1, v2, point)) {
      *out = orig + d * point.x;
    }
*/
    return (Float)point.x;
  }
  Tree(Context* in_ctx) : ctx(in_ctx) {}
};

void usage() {
  printf("usage: tetrahedralizer infile outfile [resolution] [min_quality_exp] [one_face_per_tet] [scale]\n"
         "  infile:             the input  triangle    file (wavefront .obj)          ex. in_triangle.obj\n"
         "  outfile:            the output tetrahedron file (wavefront .obj)          ex. out_tetrahedron.obj\n"
         "  [resolutin]:        interior resolution (default=10, min=0, max=100)      ex. 10\n"
         "  [min_quality_exp]:  min tet quality exp (default=-3, min = -4, max = 0)   ex. -3\n"
         "  [one_face_per_tet]: one face per tet    (default=true)                    ex. true\n"
         "  [scale]:            tet scale           (default=0.8, min=0.1, max = 1.0) ex. 0.8\n");
}

void process_args(Context* ctx, int argc, char* argv[]) {
  ctx->fn_srf_tri = std::string(argv[1]); // tri.obj
  ctx->fn_vol_tet = std::string(argv[2]); // tet.obj
  printf("infile : %s\n", ctx->fn_srf_tri.c_str());
  printf("outfile: %s\n", ctx->fn_vol_tet.c_str());
  ctx->resolution = 10;                                  // default = 10
  if (argc > 3) {
     ctx->resolution = std::atoi(argv[3]);
     ctx->resolution = std::clamp(ctx->resolution, 0, 100); // [0, 100]
     printf("[resolutin]: %d\n", ctx->resolution);
  }
  ctx->min_quality_exp = -3; // default
  if (argc > 4) {
    ctx->min_quality_exp = std::atoi(argv[4]);
    ctx->min_quality_exp = std::clamp(ctx->min_quality_exp, -4, 0); // [-4, 0]
    printf("[min_quality_exp]: %d\n", ctx->min_quality_exp);
  }
  if (argc > 5) {
    ctx->one_face_per_tet = (std::strcmp(argv[5], "false") == 0) ? false : true;
    printf("[one_face_per_tet]: %s\n", ctx->one_face_per_tet ? "true" : "false");
  }
  ctx->tet_scale = (Float)0.8;                                    // default = 0.8
  if (argc > 6) {
    ctx->tet_scale = (Float)std::atof(argv[6]);
    ctx->tet_scale = std::clamp(ctx->tet_scale, (Float)0.1, (Float)1.0); // [0.1, 1.0]
    printf("[scale]: %f\n", (float)ctx->tet_scale);
  }
}

void read_tri(Context* ctx) {
  std::string s;
  std::ifstream fs_srf_tri(ctx->fn_srf_tri);
  if (!fs_srf_tri) {
    exit(0);
  }
  while(std::getline(fs_srf_tri, s)) { // v 0.0 1.0 1.0
    double vx, vy, vz;
    int ret = sscanf(s.c_str(), "v %lf %lf %lf", &vx, &vy, &vz);
    if (ret == 3) {
      ctx->tri_vertices.push_back(Vec3(vx, vy, vz));
    }
    Triangle tri;
    int f0, f1, f2, f3, f4, f5, f6, f7, f8;
    ret = sscanf(s.c_str(), "f %d//%d %d//%d %d//%d", &f0, &f1, &f2, &f3, &f4, &f5); // f 0//1 1//1 2//2
    if (ret == 6) {
      tri.id0 = f0 - 1;
      tri.id1 = f2 - 1;
      tri.id2 = f4 - 1;
      ctx->triangles.push_back(tri);
    }
    if (ret != 6) {
      ret = sscanf(s.c_str(), "f %d/%d/%d %d/%d/%d %d/%d/%d", &f0, &f1, &f2, &f3, &f4, &f5, &f6, &f7, &f8); // f 0/0/0 1/0/0 2/0/0
      if (ret == 9) {
        tri.id0 = f0 - 1; // 1 origin to 0 origin
        tri.id1 = f3 - 1;
        tri.id2 = f6 - 1;
        ctx->triangles.push_back(tri);
      } else if (ret != 9) {
        ret = sscanf(s.c_str(), "f %d %d %d", &f0, &f1, &f2); // f 1 2 3
        if (ret == 3) {
          tri.id0 = f0 - 1;
          tri.id1 = f1 - 1;
          tri.id2 = f2 - 1;
          ctx->triangles.push_back(tri);
        }
      }
    }
  }
}

Float random_one() {
  return (Float)(rand() - 1) / RAND_MAX; // [0.0, 1.0) TODO: std::random
}

Float rand_eps() {
  Float eps = (Float)0.0001;
  return - eps + (Float)2.0 * random_one() * eps;
}

bool is_inside(Tree& tree, Vec3& p, Float min_dist = 0.0) {
  int num_in = 0;
  for(int i = 0 ; i < 6; i++) {
    auto& ctx = tree.ctx;
    for(auto& t : ctx->triangles) {
      glm::vec3 v0 = ctx->tri_vertices[t.id0];
      glm::vec3 v1 = ctx->tri_vertices[t.id1];
      glm::vec3 v2 = ctx->tri_vertices[t.id2];
      glm::vec3 point;
      glm::vec3 dirs = glm::vec3(DIRS[i].x, DIRS[i].y, DIRS[i].z);
      if (glm::intersectLineTriangle(glm::vec3(p.x, p.y, p.z), dirs, v0, v1, v2, point)) { // TODO: BVH for optimize
        if (point.x > (Float)0.0) {
          glm::vec3 n = glm::normalize(glm::cross(v1-v0, v2-v0));
          if (glm::dot(n, dirs) > (Float)0.0) {
            num_in++;
          }
          if ((min_dist > (Float)0.0) && (point.x < min_dist)) {
            return false; // too far
          }
        }
      }
    }
  }
  return (num_in > 3);
}

Vec3 get_circum_center(Vec3& p0, Vec3& p1, Vec3& p2, Vec3& p3) {
  Vec3  b   = p1 - p0;
  Vec3  c   = p2 - p0;
  Vec3  d   = p3 - p0;
  Float det = (Float)2.0 * (b.x * (c.y * d.z - c.z * d.y) - b.y * (c.x * d.z - c.z * d.x) + b.z * (c.x * d.y - c.y * d.x));
  if (std::abs(det) < std::numeric_limits<Float>::epsilon()) {
    return p0;
  } else {
    Vec3 v = glm::cross(c, d) * glm::dot(b, b) + glm::cross(d, b) * glm::dot(c, c) + glm::cross(b, c) * glm::dot(d, d);
    v /= det;
    return p0 + v;
  }
}

Float tet_quality(Vec3& p0, Vec3& p1, Vec3& p2, Vec3& p3) {
  Vec3 d0 = p1 - p0;
  Vec3 d1 = p2 - p0;
  Vec3 d2 = p3 - p0;
  Vec3 d3 = p2 - p1;
  Vec3 d4 = p3 - p2;
  Vec3 d5 = p1 - p3;
  Float s0 = glm::length(d0);
  Float s1 = glm::length(d1);
  Float s2 = glm::length(d2);
  Float s3 = glm::length(d3);
  Float s4 = glm::length(d4);
  Float s5 = glm::length(d5);
  Float ms = (s0*s0 + s1*s1 + s2*s2 + s3*s3 + s4*s4 + s5*s5) / (Float)6.0;
  Float rms = std::sqrt(ms);
  Float s   = (Float)12.0 / std::sqrt((Float)2.0);
  Float vol = glm::dot(d0, glm::cross(d1, d2)) / (Float)6.0;
  return s * vol / (rms * rms * rms); // 1.0 for regular tetrahedron
}

std::vector<int> create_tet_ids(std::vector<Vec3>& verts, Tree& tree, Float min_quality) {
  std::vector<int> tet_ids;
  int first_big = (int)verts.size() - 4;
  tet_ids.push_back(first_big);     // first big tet
  tet_ids.push_back(first_big + 1);
  tet_ids.push_back(first_big + 2);
  tet_ids.push_back(first_big + 3);
  std::vector<int> tet_marks;
  tet_marks.push_back(0);
  int tet_mark = 0;
  int first_free_tet = -1;
  std::vector<int>  neighbors;
  std::vector<Vec3>  planes_n;
  std::vector<Float> planes_d;
  for(int i = 0; i < 4; i++) {
    neighbors.push_back(-1);
    Vec3 p0 = verts[first_big + TET_FACES[i][0]];
    Vec3 p1 = verts[first_big + TET_FACES[i][1]];
    Vec3 p2 = verts[first_big + TET_FACES[i][2]];
    Vec3 n  = glm::cross(p1 - p0, p2 - p0);
    n       = glm::normalize(n);
    planes_n.push_back(n);
    planes_d.push_back(glm::dot(p0,n));
  }
  Vec3 center(0.0, 0.0, 0.0);
  printf(" ------------- tetrahedralization ------------------- \n");
  for(int i = 0; i < first_big; i++) {
    Vec3 p = verts[i];
    if (i % 100 == 0) {
      printf("inserting vert %d of %d\n", i + 1, first_big);
    }
    int  tet_nr = 0; // find non-deleted tet
    while(tet_ids[4 * tet_nr] < 0) {
      tet_nr++;
    }
    tet_mark++; // find containing tet
    bool found = false;
    while(!found) {
      if ((tet_nr < 0) || (tet_marks[tet_nr] == tet_mark)) {
        break;
      }
      tet_marks[tet_nr] = tet_mark;
      int id0 = tet_ids[4 * tet_nr + 0];
      int id1 = tet_ids[4 * tet_nr + 1];
      int id2 = tet_ids[4 * tet_nr + 2];
      int id3 = tet_ids[4 * tet_nr + 3];
      center = (verts[id0] + verts[id1] + verts[id2] + verts[id3]) * (Float)0.25;
      Float min_t = std::numeric_limits<Float>::max();
      int min_face_nr = -1;
      for (int j = 0; j < 4; j++) {
        Vec3  n  = planes_n[4 * tet_nr + j];
        Float d  = planes_d[4 * tet_nr + j];
        Float hp = glm::dot(n, p     ) - d;
        Float hc = glm::dot(n, center) - d;
        Float t = hp - hc;
        if (std::abs(t) < std::numeric_limits<Float>::epsilon()) {
          continue;
        }
        t = -hc / t; // time when c -> p hits the face
        if ((t >= 0.0) && (t < min_t)) {
          min_t = t;
          min_face_nr = j;
        }
      }
      if (min_t >= 1.0) {
        found = true;
      } else {
        tet_nr = neighbors[4 * tet_nr + min_face_nr];
      }
    }
    if (!found) {
      printf("*********** failed to insert vertex\n");
      continue;
    }
    tet_mark = tet_mark + 1; // find violating tets
    std::vector<int> violating_tets;
    std::stack<int>  st;
    st.push(tet_nr);
    while(st.size() != 0) {
      tet_nr = st.top();
      st.pop();
      if (tet_marks[tet_nr] == tet_mark) {
        continue;
      }
      tet_marks[tet_nr] = tet_mark;
      violating_tets.push_back(tet_nr);
      for(int j = 0; j < 4; j++) {
        int n = neighbors[4 * tet_nr + j];
        if ((n < 0) || (tet_marks[n] == tet_mark)) {
          continue;
        }
        int id0 = tet_ids[4 * n + 0]; // Delaunay condition test
        int id1 = tet_ids[4 * n + 1];
        int id2 = tet_ids[4 * n + 2];
        int id3 = tet_ids[4 * n + 3];
        Vec3  c = get_circum_center(verts[id0], verts[id1], verts[id2], verts[id3]);
        Float r = glm::length(verts[id0] - c);
        if (glm::length(p - c) < r) {
          st.push(n);
        }
      }
    } // remove old tets, create new nodes
    std::vector<Edge> edges;
    for(int j = 0; j < violating_tets.size(); j++) {
      tet_nr = violating_tets[j];
      std::vector<int> ids; // copy info before we delete it
      ids.resize(4);
      std::vector<int> ns;
      ns.resize(4);
      for(int k = 0; k < 4; k++) {
        ids[k] = tet_ids[4 * tet_nr + k];
        ns[k]  = neighbors[4 * tet_nr + k];
      }
      tet_ids[4 * tet_nr]     = -1; // delete the tet
      tet_ids[4 * tet_nr + 1] = first_free_tet;
      first_free_tet = tet_nr;
      for(int k = 0; k < 4; k++) { // visit neighbors
        int n = ns[k];
        if ((n >= 0) && (tet_marks[n] == tet_mark)) {
          continue;
        }
        int new_tet_nr = first_free_tet; // no neighbor or neighbor is not-violating -> we are facing the border
        if (new_tet_nr >= 0) {
          first_free_tet = tet_ids[4 * first_free_tet + 1];
        } else {
          new_tet_nr = int(tet_ids.size()) / 4;
          tet_marks.push_back(0);
          for(int l = 0; l < 4; l++) {
            tet_ids.push_back(0);
            neighbors.push_back(-1);
            planes_n.push_back(Vec3(0.0, 0.0, 0.0));
            planes_d.push_back(0.0);
          }
        }
        int id0 = ids[TET_FACES[k][2]];
        int id1 = ids[TET_FACES[k][1]];
        int id2 = ids[TET_FACES[k][0]];
        tet_ids[4 * new_tet_nr    ] = id0;
        tet_ids[4 * new_tet_nr + 1] = id1;
        tet_ids[4 * new_tet_nr + 2] = id2;
        tet_ids[4 * new_tet_nr + 3] = i;
        neighbors[4 * new_tet_nr] = n;
        if (n >= 0) {
          for(int l = 0; l < 4; l++) {
            if (neighbors[4 * n + l] == tet_nr) {
              neighbors[4 * n + l] = new_tet_nr;
            }
          }
        }
        neighbors[4 * new_tet_nr + 1] = -1; // will set the neighbors among the new tets later
        neighbors[4 * new_tet_nr + 2] = -1;
        neighbors[4 * new_tet_nr + 3] = -1;
        for(int l = 0; l < 4; l++) {
          Vec3 p0 = verts[tet_ids[4 * new_tet_nr + TET_FACES[l][0]]];
          Vec3 p1 = verts[tet_ids[4 * new_tet_nr + TET_FACES[l][1]]];
          Vec3 p2 = verts[tet_ids[4 * new_tet_nr + TET_FACES[l][2]]];
          Vec3 new_n = glm::cross(p1 - p0, p2 - p0);
          new_n = glm::normalize(new_n);
          planes_n[4 * new_tet_nr + l] = new_n;
          planes_d[4 * new_tet_nr + l] = glm::dot(new_n, p0);
        }
        if (id0 < id1) {
          edges.push_back(Edge(id0, id1, new_tet_nr, 1));
        } else {
          edges.push_back(Edge(id1, id0, new_tet_nr, 1));
        }
        if (id1 < id2) {
          edges.push_back(Edge(id1, id2, new_tet_nr, 2));
        } else {
          edges.push_back(Edge(id2, id1, new_tet_nr, 2));
        }
        if (id2 < id0) {
          edges.push_back(Edge(id2, id0, new_tet_nr, 3));
        } else {
          edges.push_back(Edge(id0, id2, new_tet_nr, 3));
        }
      } // next neighbor
    } // next violating tet
    auto comp = [](const auto& edge0, const auto& edge1) {
      return ((edge0.e[0] < edge1.e[0]) || ((edge0.e[0] == edge1.e[0]) && (edge0.e[1] < edge1.e[1])));
    };
    std::sort(edges.begin(), edges.end(), comp);
    int nr = 0;
    int num_edges = (int)edges.size();
    auto equal_edges = [](const auto& edge0, const auto& edge1) {
      return ((edge0.e[0] == edge1.e[0]) && (edge0.e[1] == edge1.e[1]));
    };
    while(nr < num_edges) {
      Edge e0 = edges[nr];
      nr++;
      if ((nr < num_edges) && (equal_edges(edges[nr], e0))) {
        Edge e1 = edges[nr];
        int id0 = tet_ids[4 * e0.e[2]];
        int id1 = tet_ids[4 * e0.e[2] + 1];
        int id2 = tet_ids[4 * e0.e[2] + 2];
        int id3 = tet_ids[4 * e0.e[2] + 3];
        int jd0 = tet_ids[4 * e1.e[2]];
        int jd1 = tet_ids[4 * e1.e[2] + 1];
        int jd2 = tet_ids[4 * e1.e[2] + 2];
        int jd3 = tet_ids[4 * e1.e[2] + 3];
        neighbors[4 * e0.e[2] + e0.e[3]] = e1.e[2];
        neighbors[4 * e1.e[2] + e1.e[3]] = e0.e[2];
        nr = nr + 1;
      }
    }
  }
  // next point
  // remove outer, deleted and outside tets
  int num_tets = ((int)tet_ids.size()) / 4;
  int num      = 0;
  int num_bad  = 0;
  for(int i = 0; i < num_tets; i++) {
    int id0 = tet_ids[4 * i];
    int id1 = tet_ids[4 * i + 1];
    int id2 = tet_ids[4 * i + 2];
    int id3 = tet_ids[4 * i + 3];
    if ( (id0 < 0) || (id0 >= first_big) || (id1 >= first_big) || (id2 >= first_big) || (id3 >= first_big) ) {
      continue;
    }
    Vec3 p0 = verts[id0];
    Vec3 p1 = verts[id1];
    Vec3 p2 = verts[id2];
    Vec3 p3 = verts[id3];
    Float quality = tet_quality(p0, p1, p2, p3);
    if (quality < min_quality) {
      num_bad++;
      continue;
    }
    center = (p0 + p1 + p2 + p3) * (Float)0.25;
    if (!is_inside(tree, center)) {
      continue;
    }
    tet_ids[num] = id0;
    num++;
    tet_ids[num] = id1;
    num++;
    tet_ids[num] = id2;
    num++;
    tet_ids[num] = id3;
    num++;
  }
  int tet_ids_size = (int)tet_ids.size();
  for(int i = 0; i < tet_ids_size - num; i++) {
    tet_ids.pop_back(); // Python -> del tet_ids[num:]
  }
  printf("%d bad tets deleted\n", num_bad);
  printf("%d tets created\n", int(tet_ids.size()) / 4);
  return tet_ids;
}

void create_tets(Context* ctx) {
  Tree tree(ctx);
  for(auto& v : ctx->tri_vertices) { // create vertices from input mesh
    v += Vec3(rand_eps(), rand_eps(), rand_eps());
    ctx->tet_vertices.push_back(v);
  }
  Vec3 center(0.0, 0.0, 0.0); // measure vertices
  constexpr Float max = std::numeric_limits<Float>::max();
  Vec3 bmin(+max, +max, +max);
  Vec3 bmax(-max, -max, -max);
  for(auto& v : ctx->tet_vertices) {
    bmin.x = std::min(bmin.x, v.x);
    bmin.y = std::min(bmin.y, v.y);
    bmin.z = std::min(bmin.z, v.z);
    bmax.x = std::max(bmax.x, v.x);
    bmax.y = std::max(bmax.y, v.y);
    bmax.z = std::max(bmax.z, v.z);
    center += v;
  }
  center /= ctx->tet_vertices.size();

  Float radius = 0.0;
  for(auto& v : ctx->tet_vertices) {
    Float d = glm::length(v - center);
    radius = std::max(radius, d);
  }
  if (ctx->resolution > 0) { // interior sampling
    Vec3  dims = bmax - bmin;
    Float dim  = std::max(dims.x, std::max(dims.y, dims.z));
    Float h    = dim / ctx->resolution;
    Vec3  v;
    for(int xi = 0; xi < int(dims.x / h) + 1; xi++) {
      v.x = bmin.x + xi * h + rand_eps();
      for(int yi = 0; yi < int(dims.y / h) + 1; yi++) {
        v.y = bmin.y + yi * h + rand_eps();
        for(int zi = 0; zi < int(dims.z / h) + 1; zi++) {
          v.z = bmin.z + zi * h + rand_eps();
          if (is_inside(tree, v, Float(0.5 * h))) {
            ctx->tet_vertices.push_back(v);
          }
        }
      }
    }
  }
  Float s = (Float)5.0 * radius; // start with 4 temporary points containing all points
  ctx->tet_vertices.push_back(Vec3( -s, 0.0, -s));
  ctx->tet_vertices.push_back(Vec3( +s, 0.0, -s));
  ctx->tet_vertices.push_back(Vec3(0.0,  +s, +s));
  ctx->tet_vertices.push_back(Vec3(0.0,  -s, +s));
  std::vector<int> faces = create_tet_ids(ctx->tet_vertices, tree, std::pow((Float)10.0, (Float)ctx->min_quality_exp));
  int num_tets = (int)faces.size() / 4;
  if (ctx->one_face_per_tet) {
    int num_src_points = (int)ctx->tri_vertices.size();
    int num_points     = (int)ctx->tet_vertices.size() - 4;
    for(int i = 0; i < num_src_points; i++) { // copy src points without distortion
      ctx->out_vertices.push_back(ctx->tri_vertices[i]);
    }
    for(int i = num_src_points; i < num_points; i++) {
      ctx->out_vertices.push_back(ctx->tet_vertices[i]);
    }
  } else {
    for(int i = 0; i < num_tets; i++) {
      auto& tvtx = ctx->tet_vertices;
      Vec3 center = (tvtx[faces[4 * i]] + tvtx[faces[4 * i + 1]] + tvtx[faces[4 * i + 2]] + tvtx[faces[4 * i + 3]]) * (Float)0.25;
      for(int j = 0; j < 4; j++) {
        for(int k = 0; k < 3; k++) {
          Vec3 p = tvtx[faces[4 * i + TET_FACES[j][k]]];
          p = center + (p - center) * ctx->tet_scale;
          ctx->out_vertices.push_back(p);
        }
      }
    }
  }
  int nr = 0;
  for(int i = 0; i < num_tets; i++) {
    if (ctx->one_face_per_tet) {
      Tetrahedron tet;
      tet.id0 = faces[4 * i];
      tet.id1 = faces[4 * i + 1];
      tet.id2 = faces[4 * i + 2];
      tet.id3 = faces[4 * i + 3];
      ctx->tetrahedron_faces.push_back(tet);
    } else {
      for(int j = 0; j < 4; j++) {
        Triangle tet;
        tet.id0 = nr;
        tet.id1 = nr + 1;
        tet.id2 = nr + 2;
        ctx->triangle_faces.push_back(tet);
        nr += 3;
      }
    }
  }
}

void write_tet(Context* ctx) {
  FILE* fp = fopen(ctx->fn_vol_tet.c_str(), "w");
  if (!fp) { exit(0); } // can't open
  for(auto& v : ctx->out_vertices) {
    fprintf(fp, "v %lf %lf %lf\n", v.x, v.y, v.z);
  }
  for(auto& t : ctx->tetrahedron_faces) {
    fprintf(fp, "f %d// %d// %d// %d//\n", t.id0 + 1, t.id1 + 1, t.id2 + 1, t.id3 + 1);
  }
  for(auto& t : ctx->triangle_faces) {
    fprintf(fp, "f %d// %d// %d//\n", t.id0 + 1, t.id1 + 1, t.id2 + 1);
  }
  fclose(fp);
}

int main(int argc, char* argv[]) {
  Context* ctx = new Context();
  if (argc < 3) {
    usage();
  } else {
    process_args(ctx, argc, argv);
    read_tri(ctx);
    create_tets(ctx);
    write_tet(ctx);
  }
  if (ctx) { delete ctx; }
  return 0;
}