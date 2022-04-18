//
// LICENSE:
//
// Copyright (c) 2016 -- 2020 Fabio Pellacini
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <yocto/yocto_commonio.h>
#include <yocto/yocto_image.h>
#include <yocto/yocto_math.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>

#include <iostream>
using namespace yocto::math;
namespace sio = yocto::sceneio;
namespace shp = yocto::shape;
namespace cli = yocto::commonio;

#include <memory>
using std::string;
using namespace std::string_literals;

#include "ext/filesystem.hpp"
namespace sfs = ghc::filesystem;

#include "ext/perlin-noise/noise1234.h"

float noise(const vec3f& p) { return noise3(p.x, p.y, p.z); }
vec2f noise2(const vec3f& p) {
  return {noise(p + vec3f{0, 0, 0}), noise(p + vec3f{3, 7, 11})};
}
vec3f noise3(const vec3f& p) {
  return {noise(p + vec3f{0, 0, 0}), noise(p + vec3f{3, 7, 11}),
      noise(p + vec3f{13, 17, 19})};
}
float fbm(const vec3f& p, int octaves) {
  float calcNoise = 0.0f;
  for (int i = 0; i < octaves; i++)
    calcNoise += pow(2.f, -i) * noise(pow(2.f, i) * p);
  return calcNoise;
}
float turbulence(const vec3f& p, int octaves) {
  float calcNoise = 0.f;
  for (int i = 0; i < octaves; i++)
    calcNoise += pow(2.f, -i) * yocto::math::abs(noise(pow(2.f, i) * p));
  return calcNoise;
}

float ridge(const vec3f& p, int octaves) {
  // return yocto::math::perlin_ridge(p);
  float calcNoise = 0.f;
  for (int i = 0; i <= octaves; i++)
    calcNoise += yocto::math::pow(2.f, -i) *
                 yocto::math::pow(
                     1.f - yocto::math::abs(noise(pow(2.f, i) * p)), 2.f) /
                 2.f;
  return calcNoise;
}

sio::object* get_object(sio::model* scene, const std::string& name) {
  for (auto object : scene->objects)
    if (object->name == name) return object;
  cli::print_fatal("unknown object " + name);
  return nullptr;
}

void add_polyline(sio::shape* shape, const std::vector<vec3f>& positions,
    const std::vector<vec3f>& colors, float thickness = 0.0001f) {
  auto offset = (int)shape->positions.size();
  shape->positions.insert(
      shape->positions.end(), positions.begin(), positions.end());
  shape->colors.insert(shape->colors.end(), colors.begin(), colors.end());
  shape->radius.insert(shape->radius.end(), positions.size(), thickness);
  for (auto idx = 0; idx < positions.size() - 1; idx++) {
    shape->lines.push_back({offset + idx, offset + idx + 1});
  }
}

void sample_shape(std::vector<vec3f>& positions, std::vector<vec3f>& normals,
    std::vector<vec2f>& texcoords, sio::shape* shape, int num) {
  auto triangles  = shape->triangles;
  auto qtriangles = shp::quads_to_triangles(shape->quads);
  triangles.insert(triangles.end(), qtriangles.begin(), qtriangles.end());
  auto cdf = shp::sample_triangles_cdf(triangles, shape->positions);
  auto rng = make_rng(19873991);
  for (auto idx = 0; idx < num; idx++) {
    auto [elem, uv] = shp::sample_triangles(cdf, rand1f(rng), rand2f(rng));
    auto q          = triangles[elem];
    positions.push_back(interpolate_triangle(shape->positions[q.x],
        shape->positions[q.y], shape->positions[q.z], uv));
    normals.push_back(normalize(interpolate_triangle(
        shape->normals[q.x], shape->normals[q.y], shape->normals[q.z], uv)));
    if (!texcoords.empty()) {
      texcoords.push_back(interpolate_triangle(shape->texcoords[q.x],
          shape->texcoords[q.y], shape->texcoords[q.z], uv));
    } else {
      texcoords.push_back(uv);
    }
  }
}

struct terrain_params {
  float size    = 0.1f;
  vec3f center  = zero3f;
  float height  = 0.1f;
  float scale   = 10;
  int   octaves = 8;
  vec3f bottom  = srgb_to_rgb(vec3f{154, 205, 50} / 255);
  vec3f middle  = srgb_to_rgb(vec3f{205, 133, 63} / 255);
  vec3f top     = srgb_to_rgb(vec3f{240, 255, 255} / 255);
};

void make_terrain(
    sio::model* scene, sio::object* object, const terrain_params& params) {
  for (int i = 0; i < object->shape->positions.size(); i++) {
    auto& pos = object->shape->positions[i];
    pos += object->shape->normals[i] *
           (ridge(pos * params.scale, params.octaves) * params.height *
               (1 - yocto::math::length(pos - params.center) / params.size));

    // coloring
    auto percentage = pos.y / params.height * 100.f;
    if (percentage <= 30.f)
      object->shape->colors.push_back(params.bottom);
    else if (percentage <= 60.f)
      object->shape->colors.push_back(params.middle);
    else
      object->shape->colors.push_back(params.top);
  }
  yocto::shape::update_normals(
      object->shape->normals, object->shape->quads, object->shape->positions);
}

struct displacement_params {
  float height  = 0.02f;
  float scale   = 50;
  int   octaves = 8;
  vec3f bottom  = srgb_to_rgb(vec3f{64, 224, 208} / 255);
  vec3f top     = srgb_to_rgb(vec3f{244, 164, 96} / 255);
};

void make_displacement(
    sio::model* scene, sio::object* object, const displacement_params& params) {
  for (int i = 0; i < object->shape->positions.size(); i++) {
    auto& pos   = object->shape->positions[i];
    auto  lastP = pos;
    pos += object->shape->normals[i] *
           (turbulence(pos * params.scale, params.octaves) * params.height);
    // interpolo per prendere il colore
    object->shape->colors.push_back(yocto::math::interpolate_line(params.bottom,
        params.top, yocto::math::distance(pos, lastP) / params.height));
  }
  yocto::shape::update_normals(
      object->shape->normals, object->shape->quads, object->shape->positions);
}

struct hair_params {
  int   num      = 100000;
  int   steps    = 1;
  float lenght   = 0.02f;
  float scale    = 250;
  float strength = 0.01f;
  float gravity  = 0.0f;
  vec3f bottom   = srgb_to_rgb(vec3f{25, 25, 25} / 255);
  vec3f top      = srgb_to_rgb(vec3f{244, 164, 96} / 255);
};

using namespace yocto::shape;

void make_hair(sio::model* scene, sio::object* object, sio::object* hair,
    const hair_params& params) {
  yocto::sceneio::shape shape;
  hair->shape       = add_shape(scene);
  auto positionSize = object->shape->positions.size();
  //genero i punti per i capelli
  sample_shape(object->shape->positions, object->shape->normals,
      object->shape->texcoords, object->shape, params.num);
  for (int i = positionSize; i < object->shape->positions.size(); i++) {
    // pos del capello
    auto pos = object->shape->positions[i];
    // direzione del capello
    auto               normal = object->shape->normals[i];
    std::vector<vec3f> points;
    std::vector<vec3f> colors;
    auto               startPos = pos;
    // niente noise al primo punto
    points.push_back(pos);
    //interpolo per prendere il colore
    colors.push_back(yocto::math::interpolate_line(params.bottom, params.top,
        yocto::math::distance(pos, startPos) / params.lenght));
    //creo i vari segmenti del capello
    for (int s = 0; s < params.steps; s++) {
      auto oldPos = pos;
      pos += normal * (params.lenght / params.steps) +
             noise3(pos * params.scale) * params.strength;
      pos.y -= params.gravity;
      normal = normalize(pos - oldPos);
      points.push_back(pos);
      colors.push_back(yocto::math::interpolate_line(params.bottom, params.top,
          yocto::math::distance(pos, startPos) / params.lenght));
    }
    //aggiungo il colore alla punta e attacco il capello alla shape
    colors[params.steps] = params.top;
    add_polyline(hair->shape, points, colors);
  }
  //aggiorno le tangenti
  for (auto tangent : yocto::shape::compute_tangents(
           hair->shape->lines, hair->shape->positions))
    hair->shape->tangents.push_back(vec4f{tangent.x, tangent.y, tangent.z, 1});
}

struct grass_params {
  int num = 10000;
};

void make_grass(sio::model* scene, sio::object* object,
    const std::vector<sio::object*>& grasses, const grass_params& params) {
  auto rng = make_rng(198767);
  for (auto grass : grasses) grass->instance = add_instance(scene);
  //#############################
  sample_shape(object->shape->positions, object->shape->normals,
      object->shape->texcoords, object->shape, params.num);
  for (int i = 0; i < object->shape->positions.size(); i++) {
    auto& pos = object->shape->positions[i];
    // creo un filo d'erba random e da grasses assegno shape a material
    auto grass      = add_object(scene);
    int  index      = rand1i(rng, grasses.size());
    grass->shape    = grasses[index]->shape;
    grass->material = grasses[index]->material;
    float random = 0.9f + rand1f(rng) * 0.1f;
    // traslo
    grass->frame *= yocto::math::translation_frame(pos);
    //ruoto prima su z e poi su y
    grass->frame *= yocto::math::scaling_frame({random, random, random});
    random = rand1f(rng) * 2.f * pif;
    grass->frame *= yocto::math::rotation_frame(grass->frame.y, random);
    random = 0.1f + rand1f(rng) * 0.1f;
    grass->frame *= yocto::math::rotation_frame(grass->frame.z, random);
  }
}

void make_dir(const std::string& dirname) {
  if (sfs::exists(dirname)) return;
  try {
    sfs::create_directories(dirname);
  } catch (...) {
    cli::print_fatal("cannot create directory " + dirname);
  }
}

int main(int argc, const char* argv[]) {
  // command line parameters
  auto terrain      = ""s;
  auto tparams      = terrain_params{};
  auto displacement = ""s;
  auto dparams      = displacement_params{};
  auto hair         = ""s;
  auto hairbase     = ""s;
  auto hparams      = hair_params{};
  auto grass        = ""s;
  auto grassbase    = ""s;
  auto gparams      = grass_params{};
  auto output       = "out.json"s;
  auto filename     = "scene.json"s;

  // parse command line
  auto cli = cli::make_cli("yscenegen", "Make procedural scenes");
  add_option(cli, "--terrain", terrain, "terrain object");
  add_option(cli, "--displacement", displacement, "displacement object");
  add_option(cli, "--hair", hair, "hair object");
  add_option(cli, "--hairbase", hairbase, "hairbase object");
  add_option(cli, "--grass", grass, "grass object");
  add_option(cli, "--grassbase", grassbase, "grassbase object");
  add_option(cli, "--hairnum", hparams.num, "hair number");
  add_option(cli, "--hairlen", hparams.lenght, "hair length");
  add_option(cli, "--hairstr", hparams.strength, "hair strength");
  add_option(cli, "--hairgrav", hparams.gravity, "hair gravity");
  add_option(cli, "--hairstep", hparams.steps, "hair steps");
  add_option(cli, "--output,-o", output, "output scene");
  add_option(cli, "scene", filename, "input scene", true);
  parse_cli(cli, argc, argv);

  // load scene
  auto scene_guard = std::make_unique<sio::model>();
  auto scene       = scene_guard.get();
  auto ioerror     = ""s;
  if (!load_scene(filename, scene, ioerror, cli::print_progress))
    cli::print_fatal(ioerror);

  // create procedural geometry
  if (terrain != "") {
    make_terrain(scene, get_object(scene, terrain), tparams);
  }
  if (displacement != "") {
    make_displacement(scene, get_object(scene, displacement), dparams);
  }
  if (hair != "") {
    make_hair(
        scene, get_object(scene, hairbase), get_object(scene, hair), hparams);
  }
  if (grass != "") {
    auto grasses = std::vector<sio::object*>{};
    for (auto object : scene->objects)
      if (object->name.find(grass) != scene->name.npos)
        grasses.push_back(object);
    make_grass(scene, get_object(scene, grassbase), grasses, gparams);
  }

  // make a directory if needed
  make_dir(sfs::path(output).parent_path());
  if (!scene->shapes.empty())
    make_dir(sfs::path(output).parent_path() / "shapes");
  if (!scene->subdivs.empty())
    make_dir(sfs::path(output).parent_path() / "subdivs");
  if (!scene->textures.empty())
    make_dir(sfs::path(output).parent_path() / "textures");
  if (!scene->instances.empty())
    make_dir(sfs::path(output).parent_path() / "instances");

  // save scene
  if (!save_scene(output, scene, ioerror, cli::print_progress))
    cli::print_fatal(ioerror);

  // done
  return 0;
}
