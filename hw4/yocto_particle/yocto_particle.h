//
// Yocto/Particle: Demo project for physically-based animation.
//

//
// LICENSE:
//
// Copyright (c) 2020 -- 2020 Fabio Pellacini
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//

#ifndef _YOCTO_PARTICLE_
#define _YOCTO_PARTICLE_

// -----------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------

#include <yocto/yocto_math.h>
#include <yocto/yocto_shape.h>

#include <string>

// -----------------------------------------------------------------------------
// ALIASES
// -----------------------------------------------------------------------------
namespace yocto::particle {

// Namespace aliases
namespace par = yocto::particle;
namespace shp = yocto::shape;

// Math defitions
using math::bbox3f;
using math::byte;
using math::frame3f;
using math::identity3x4f;
using math::mat4f;
using math::rng_state;
using math::uint;
using math::vec2f;
using math::vec2i;
using math::vec3b;
using math::vec3f;
using math::vec3i;
using math::vec4b;
using math::vec4f;
using math::vec4i;
using math::zero2f;
using math::zero2i;
using math::zero3f;
using math::zero3i;

}  // namespace yocto::particle

// -----------------------------------------------------------------------------
// SIMULATION DATA AND API
// -----------------------------------------------------------------------------
namespace yocto::particle {

// Springs
struct spring {
  int   vert0 = -1;
  int   vert1 = -1;
  float rest  = 0;
  float coeff = 0;
};

// Collisions
struct collision {
  int   vert     = 0;
  vec3f position = {0, 0, 0};
  vec3f normal   = {0, 0, 0};
};

// Simulation shape
struct shape {
  // particle data
  std::vector<vec3f> positions  = {};
  std::vector<vec3f> normals    = {};
  std::vector<float> radius     = {};
  std::vector<float> invmass    = {};
  std::vector<vec3f> velocities = {};

  // material data
  float spring_coeff = 0;

  // particle emitter
  vec3f     emit_velocity = {0, 0, 0};
  float     emit_rngscale = 0;
  rng_state emit_rng      = {};

  // shape data
  std::vector<int>   points    = {};
  std::vector<vec2i> lines     = {};
  std::vector<vec3i> triangles = {};
  std::vector<vec4i> quads     = {};

  // simulation data
  std::vector<vec3f>     old_positions = {};
  std::vector<vec3f>     forces        = {};
  std::vector<spring>    springs       = {};
  std::vector<float>     lambdas       = {};
  std::vector<collision> collisions    = {};

  // initial configuration to reply animation
  std::vector<vec3f> initial_positions  = {};
  std::vector<vec3f> initial_normals    = {};
  std::vector<vec3f> initial_velocities = {};
  std::vector<float> initial_invmass    = {};
  std::vector<float> initial_radius     = {};
  std::vector<int>   initial_pinned     = {};
};

// Simulation collider
struct collider {
  // Vertex data
  std::vector<vec3f> positions = {};
  std::vector<vec3f> normals   = {};
  std::vector<float> radius    = {};

  // Element data
  std::vector<vec3i> triangles = {};
  std::vector<vec4i> quads     = {};

  // bvh
  shp::bvh_tree bvh = {};
};

// Simulation scene
struct scene {
  std::vector<par::shape*>    shapes    = {};
  std::vector<par::collider*> colliders = {};
  ~scene();
};

// Solver type
enum struct solver_type { mass_spring, position_based };

// Solver names
const auto solver_names = std::vector<std::string>{
    "mass_spring", "position_based"};

// Simulation parameters
struct simulation_params {
  solver_type solver       = solver_type::mass_spring;
  float       gravity      = 9.8;
  float       deltat       = 0.5 * 1.0 / 60.0;
  int         mssteps      = 200;
  int         pdbsteps     = 100;
  int         frames       = 120;
  float       initvelocity = 0;
  float       dumping      = 2;
  float       minvelocity  = 0.01;
  vec2f       bounce       = {0.05f, 1};
  int         seed         = 987121;
};

// Initialize the simulation state
void init_simulation(par::scene* scene, const simulation_params& params);

// Simulate one frame
void simulate_frame(par::scene* scene, const simulation_params& params);

// Progress report callback
using progress_callback =
    std::function<void(const std::string& message, int current, int total)>;

// Simulate the whole sequence
void simulate_frames(par::scene* scene, const simulation_params& params,
    progress_callback progress_cb);

// Scene creation
par::shape* add_shape(par::scene* scene);

// Scene creation
par::shape*    add_particles(par::scene* scene, const std::vector<int>& points,
       const std::vector<vec3f>& positions, const std::vector<float>& radius,
       float mass, float random_velocity);
par::shape*    add_cloth(par::scene* scene, const std::vector<vec4i>& quads,
       const std::vector<vec3f>& positions, const std::vector<vec3f>& normals,
       const std::vector<float>& radius, float mass, float coeff,
       const std::vector<int>& pinned);
par::collider* add_collider(par::scene* scene,
    const std::vector<vec3i>& triangles, const std::vector<vec4i>& quads,
    const std::vector<vec3f>& positions, const std::vector<vec3f>& normals,
    const std::vector<float>& radius);

// Get shape properties
void get_positions(par::shape* shape, std::vector<vec3f>& positions);
void get_normals(par::shape* shape, std::vector<vec3f>& normals);

}  // namespace yocto::particle

#endif
