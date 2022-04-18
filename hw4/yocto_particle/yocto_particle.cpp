//
// Implementation for Yocto/Particle.
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

#include "yocto_particle.h"

#include <yocto/yocto_shape.h>

#include <unordered_set>

// -----------------------------------------------------------------------------
// ALIASES
// -----------------------------------------------------------------------------
namespace yocto::particle {

// import math symbols for use
using math::abs;
using math::acos;
using math::atan;
using math::atan2;
using math::clamp;
using math::cos;
using math::exp;
using math::exp2;
using math::flt_max;
using math::fmod;
using math::log;
using math::log2;
using math::make_rng;
using math::max;
using math::min;
using math::perspective_mat;
using math::pow;
using math::ray3f;
using math::sin;
using math::sqrt;
using math::tan;

}  // namespace yocto::particle

// -----------------------------------------------------------------------------
// SIMULATION DATA AND API
// -----------------------------------------------------------------------------
namespace yocto::particle {

// cleanup
scene::~scene() {
    for (auto shape : shapes) delete shape;
    for (auto collider : colliders) delete collider;
}

// Scene creation
par::shape* add_shape(par::scene* scene) {
    return scene->shapes.emplace_back(new par::shape{});
}
par::collider* add_collider(par::scene* scene) {
    return scene->colliders.emplace_back(new par::collider{});
}
par::shape* add_particles(par::scene* scene, const std::vector<int>& points,
    const std::vector<vec3f>& positions, const std::vector<float>& radius,
    float mass, float random_velocity) {
    auto shape               = add_shape(scene);
    shape->points            = points;
    shape->initial_positions = positions;
    shape->initial_normals.assign(shape->positions.size(), {0, 0, 1});
    shape->initial_radius = radius;
    shape->initial_invmass.assign(
        positions.size(), 1 / (mass * positions.size()));
    shape->initial_velocities.assign(positions.size(), {0, 0, 0});
    shape->emit_rngscale = random_velocity;
    return shape;
}
par::shape* add_cloth(par::scene* scene, const std::vector<vec4i>& quads,
    const std::vector<vec3f>& positions, const std::vector<vec3f>& normals,
    const std::vector<float>& radius, float mass, float coeff,
    const std::vector<int>& pinned) {
    auto shape               = add_shape(scene);
    shape->quads             = quads;
    shape->initial_positions = positions;
    shape->initial_normals   = normals;
    shape->initial_radius    = radius;
    shape->initial_invmass.assign(
        positions.size(), 1 / (mass * positions.size()));
    shape->initial_velocities.assign(positions.size(), {0, 0, 0});
    shape->initial_pinned = pinned;
    shape->spring_coeff   = coeff;
    return shape;
}
par::collider* add_collider(par::scene* scene,
    const std::vector<vec3i>& triangles, const std::vector<vec4i>& quads,
    const std::vector<vec3f>& positions, const std::vector<vec3f>& normals,
    const std::vector<float>& radius) {
    auto collider       = add_collider(scene);
    collider->quads     = quads;
    collider->triangles = triangles;
    collider->positions = positions;
    collider->normals   = normals;
    collider->radius    = radius;
    return collider;
}

// Set shapes
void set_velocities(
    par::shape* shape, const vec3f& velocity, float random_scale) {
    shape->emit_velocity = velocity;
    shape->emit_rngscale = random_scale;
}

// Get shape properties
void get_positions(par::shape* shape, std::vector<vec3f>& positions) {
    positions = shape->positions;
}
void get_normals(par::shape* shape, std::vector<vec3f>& normals) {
    normals = shape->normals;
}

}  // namespace yocto::particle

// -----------------------------------------------------------------------------
// SIMULATION DATA AND API
// -----------------------------------------------------------------------------
namespace yocto::particle {

// Init simulation
void init_simulation(par::scene* scene, const simulation_params& params) {
    auto sid = 0;
    for (auto shape : scene->shapes) {
        shape->emit_rng = make_rng(params.seed, (sid++) * 2 + 1);
        // -------------------------------------------------- intitialize state
        shape->positions = shape->initial_positions;
        shape->normals   = shape->initial_normals;
        shape->invmass   = shape->initial_invmass;
        shape->radius    = shape->initial_radius;
        // inizializzo il vettore forces
        shape->forces = std::vector<vec3f>(shape->positions.size(), zero3f);
        // initialize pinned
        for (auto pin : shape->initial_pinned) shape->invmass[pin] = 0.f;
        // initialize velocities
        shape->velocities = shape->initial_velocities;
        for (int i = 0; i < shape->initial_velocities.size(); i++) {
            auto s = yocto::math::sample_sphere(math::rand2f(shape->emit_rng));
            // aggiungo una velocità random
            shape->velocities[i] += s * shape->emit_rngscale *
                                    math::rand1f(shape->emit_rng);
        }

        // initialize springs

        // clear delle molle
        shape->springs.clear();
        if (shape->spring_coeff > 0) {
            // lati dei quads
            for (auto edge : yocto::shape::get_edges(shape->quads))
                shape->springs.push_back({edge.x, edge.y,
                    yocto::math::distance(
                        shape->positions[edge.x], shape->positions[edge.y]),
                    shape->spring_coeff});
            // diagonali
            for (auto quad : shape->quads) {
                shape->springs.push_back({quad.y, quad.w,
                    yocto::math::distance(
                        shape->positions[quad.y], shape->positions[quad.w]),
                    shape->spring_coeff});
                shape->springs.push_back({quad.x, quad.z,
                    yocto::math::distance(
                        shape->positions[quad.x], shape->positions[quad.z]),
                    shape->spring_coeff});
            }
        }
    }
    for (auto collider : scene->colliders) {
        // intiialize bvh
        // controllo se ho quads o tri
        if (!collider->triangles.empty())
            yocto::shape::make_triangles_bvh(collider->bvh, collider->triangles,
                collider->positions, collider->radius);
        else
            yocto::shape::make_quads_bvh(collider->bvh, collider->quads,
                collider->positions, collider->radius);
    }
}

// check if a point is inside a collider
bool collide_collider(par::collider* collider, const vec3f& position,
    vec3f& hit_position, vec3f& hit_normal) {
    // raggio verso l'alto partendo dal punto
    auto ray = ray3f(position, {0, 1, 0});
    // controllo se ho quad o tri
    if (!collider->quads.empty()) {
        // quads
        // se non c'è intersezione non c'è collisione
        auto isec = yocto::shape::intersect_quads_bvh(
            collider->bvh, collider->quads, collider->positions, ray);
        if (!isec.hit) return false;
        // altrimenti calcolo la posizione del punto di intersez e la normale
        auto quad    = collider->quads[isec.element];
        hit_position = math::interpolate_quad(collider->positions[quad.x],
            collider->positions[quad.y], collider->positions[quad.z],
            collider->positions[quad.w], isec.uv);
        hit_normal   = math::normalize(math::interpolate_quad(
            collider->normals[quad.x], collider->normals[quad.y],
            collider->normals[quad.z], collider->normals[quad.w], isec.uv));
        return math::dot(hit_normal, ray.d) > 0;

    } else {
        // triangles
        // se non c'è intersezione non c'è collisione
        auto isec = yocto::shape::intersect_triangles_bvh(
            collider->bvh, collider->triangles, collider->positions, ray);
        if (!isec.hit) return false;
        // altrimenti calcolo la posizione del punto di intersez e la normale
        auto triangle = collider->triangles[isec.element];
        hit_position  = math::interpolate_triangle(
            collider->positions[triangle.x], collider->positions[triangle.y],
            collider->positions[triangle.z], isec.uv);
        hit_normal = math::normalize(math::interpolate_triangle(
            collider->normals[triangle.x], collider->normals[triangle.y],
            collider->normals[triangle.z], isec.uv));
        return math::dot(hit_normal, ray.d) > 0;
    }
    return false;
}

// simulate mass-spring
void simulate_massspring(par::scene* scene, const simulation_params& params) {
    // SAVE OLD POSITIONS
    for (auto& particle : scene->shapes) {
        particle->old_positions = particle->positions;
    }
    // COMPUTE DYNAMICS
    for (int s = 0; s < params.mssteps; s++) {
        auto ddt = params.deltat / params.mssteps;
        // COMPUTE FORCES
        for (auto& particle : scene->shapes) {
            for (int i = 0; i < particle->invmass.size(); i++) {
                // se invmass è 0 il punto è pinned, skippo
                if (particle->invmass[i] == 0.f) continue;
                particle->forces[i] = vec3f{0, -params.gravity, 0} /
                                      particle->invmass[i];
            }
            for (auto& spring : particle->springs) {
                // prendo i due punti agli estremi della molla
                auto& particle0 = spring.vert0;
                auto& particle1 = spring.vert1;
                // calcolo l'invmass
                auto invmass = particle->invmass[particle0] +
                               particle->invmass[particle1];
                // se è 0 è pinned, skippo
                if (invmass == 0.f) continue;
                // differenza delle pos
                auto delta_pos = particle->positions[particle1] -
                                 particle->positions[particle0];
                // direzione molla
                auto spring_dir = yocto::math::normalize(delta_pos);
                // lunghezza molla
                auto spring_len = yocto::math::length(delta_pos);
                // calcolo la forza della molla
                auto force = spring_dir * (spring_len / spring.rest - 1.f) /
                             (spring.coeff * invmass);
                auto delta_vel = particle->velocities[particle1] -
                                 particle->velocities[particle0];
                force += yocto::math::dot(delta_vel / spring.rest, spring_dir) *
                         spring_dir / (1000.f * invmass * spring.coeff);
                particle->forces[particle0] += force;
                particle->forces[particle1] -= force;
            }
            // UPDATE STATE
            for (int i = 0; i < particle->invmass.size(); i++) {
                if (particle->invmass[i] == 0.f) continue;
                particle->velocities[i] += ddt * particle->forces[i] *
                                           particle->invmass[i];
                particle->positions[i] += ddt * particle->velocities[i];
            }
        }
    }
    for (auto& particle : scene->shapes) {
        // HANDLE COLLISIONS
        for (int i = 0; i < particle->invmass.size(); i++) {
            if (particle->invmass[i] == 0.f) continue;
            for (auto collider : scene->colliders) {
                auto hitpos     = zero3f;
                auto hit_normal = zero3f;
                if (collide_collider(
                        collider, particle->positions[i], hitpos, hit_normal)) {
                    // la particella è 'entrata' in un oggetto, correggo
                    particle->positions[i] = hitpos + hit_normal * 0.005f;
                    auto projection        = math::dot(
                        hit_normal, particle->velocities[i]);
                    particle->velocities[i] =
                        (particle->velocities[i] - projection * hit_normal) *
                            (1.f - params.bounce.x) -
                        projection * hit_normal * (1.f - params.bounce.y);
                }
            }
        }
        // ADJUST VELOCITY
        for (int i = 0; i < particle->invmass.size(); i++) {
            if (particle->invmass[i] == 0.f) continue;
            // dumping
            particle->velocities[i] *= (1.f - params.dumping * params.deltat);
            // sleeping
            // se la velocità è inferiore ad un minimo settato la particella
            // va in sleeping state
            if (math::length(particle->velocities[i]) < params.minvelocity)
                particle->velocities[i] = zero3f;
        }
        // RECOMPUTE NORMALS
        if (!particle->quads.empty())
            particle->normals = yocto::shape::compute_normals(
                particle->quads, particle->positions);
        else
            particle->normals = yocto::shape::compute_normals(
                particle->triangles, particle->positions);
    }
}

// simulate pbd
void simulate_pbd(par::scene* scene, const simulation_params& params) {
    // SAVE OLD POSITOINS
    for (auto& particle : scene->shapes) {
        particle->old_positions = particle->positions;
    }
    // PREDICT POSITIONS
    for (auto& particle : scene->shapes) {
        for (int i = 0; i < particle->invmass.size(); i++) {
            if (particle->invmass[i] == 0.f) continue;
            // predict position con semi-implicit euler
            particle->velocities[i] += vec3f{0, -params.gravity, 0} *
                                       params.deltat;
            particle->positions[i] += particle->velocities[i] * params.deltat;
        }
    }
    // DETECT COLLISIONS
    // aggiusto le posiz dei vertici per rispondere alla collisione
    // prima rilevo tutte le collision poi in solve constraints le risolvo
    for (auto& particle : scene->shapes) {
        particle->collisions.clear();
        for (int i = 0; i < particle->invmass.size(); i++) {
            if (particle->invmass[i] == 0.f) continue;
            // per ogni particella provo tutti i collider
            for (auto collider : scene->colliders) {
                auto hit_position = zero3f;
                auto hit_normal   = zero3f;
                // se non c'è collisione -> skip
                if (!collide_collider(collider, particle->positions[i],
                        hit_position, hit_normal))
                    continue;
                // altrimenti salvo la collision nell'array
                particle->collisions.push_back({i, hit_position, hit_normal});
            }
        }
    }
    // SOLVE CONSTRAINTS
    for (int s = 0; s < params.pdbsteps; s++) {
        for (auto& particle : scene->shapes) {
            // SOLVE SPRINGS
            for (auto& spring : particle->springs) {
                int  p0      = spring.vert0;
                int  p1      = spring.vert1;
                auto invmass = particle->invmass[p0] + particle->invmass[p1];
                if (invmass == 0.f) continue;
                // sposto ogni particella di una quantità inversamente
                // proporzionale alla sua massa le particelle più pesanti si
                // spostano meno, ecc..
                auto dir = particle->positions[p1] -
                           particle->positions[p0];  // direz lungo la quale
                                                     // sposto le particelle
                auto len = math::length(dir);        // distanza corrente
                dir /= len;                          // normalizzo la dir
                // calcolo la quantità dello spostamento
                auto lambda = (1 - spring.coeff) * (len - spring.rest) /
                              invmass;
                particle->positions[p0] += particle->invmass[p0] * lambda * dir;
                particle->positions[p1] -= particle->invmass[p1] * lambda * dir;
            }
            // SOLVE COLLISIONS
            for (auto& collision : particle->collisions) {
                auto p = collision.vert;
                if (particle->invmass[p] == 0.f) continue;
                // calcolo la proiezione della posiz corrente - posiz della
                // collisione, rispetto alla normale della coll
                auto projection = dot(
                    particle->positions[p] - collision.position,
                    collision.normal);
                // se è positivo allora la particella è sopra il punto di
                // collisione(non all'interno dell'oggetto)

                if (projection >= 0) continue;
                // altrimenti è entrata nell'oggetto, sposto lungo la normale di
                // una quantità pari a quanto sono entrato
                particle->positions[p] += -projection * collision.normal;
            }
        }
    }
    // COMPUTE VELOCITIES
    for (auto& particle : scene->shapes) {
        for (int i = 0; i < particle->invmass.size(); i++) {
            if (particle->invmass[i] == 0.f) continue;
            particle->velocities[i] =
                (particle->positions[i] - particle->old_positions[i]) /
                params.deltat;
        }
    }
    // ADJUST VELOCITY
    for (auto& particle : scene->shapes) {
        for (int i = 0; i < particle->invmass.size(); i++) {
            if (particle->invmass[i] == 0.f) continue;
            particle->velocities[i] *= (1.f - params.dumping * params.deltat);
            // sleep
            if (math::length(particle->velocities[i]) < params.minvelocity)
                particle->velocities[i] = zero3f;
        }
        // RECOMPUTE NORMALS
        if(particle->quads.empty())
            particle->normals = yocto::shape::compute_normals(particle->triangles, particle->positions);
        else
            particle->normals = yocto::shape::compute_normals(particle->quads, particle->positions);
    }
}

// Simulate one step
void simulate_frame(par::scene* scene, const simulation_params& params) {
    switch (params.solver) {
        case solver_type::mass_spring:
            return simulate_massspring(scene, params);
        case solver_type::position_based: return simulate_pbd(scene, params);
        default: throw std::invalid_argument("unknown solver");
    }
}

// Simulate the whole sequence
void simulate_frames(par::scene* scene, const simulation_params& params,
    progress_callback progress_cb) {
    // handle progress
    auto progress = vec2i{0, 1 + (int)params.frames};

    if (progress_cb) progress_cb("init simulation", progress.x++, progress.y);
    init_simulation(scene, params);

    for (auto idx = 0; idx < params.frames; idx++) {
        if (progress_cb)
            progress_cb("simulate frames", progress.x++, progress.y);
        simulate_frame(scene, params);
    }

    if (progress_cb) progress_cb("simulate frames", progress.x++, progress.y);
}

}  // namespace yocto::particle
