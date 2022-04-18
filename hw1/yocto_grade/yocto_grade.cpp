//
// Implementation for Yocto/Grade.
//

//
// LICENSE:
//
// Copyright (c) 2020 -- 2020 Fabio Pellacini
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

#include "yocto_grade.h"
#include <iostream>
// -----------------------------------------------------------------------------
// COLOR GRADING FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto::grade {

void exposure(math::vec3f& newPixel, const math::vec3f& oldPixel, float exposure)
{
  newPixel = oldPixel * exp2(exposure);
}

void filmic(math::vec3f& pixel)
{
  /* c *= 0.6; c = (c^2 * 2.51 + c * 0.03) / (c^2 * 2.43 + c * 0.59 + 0.14) */
  pixel = pixel * 0.6;
  pixel = (pow(pixel, 2) * 2.51 + pixel * 0.03) / (pow(pixel, 2) * 2.43 + pixel * 0.59 + 0.14);
}

void srgb(math::vec3f& pixel)
{
  pixel = pow(pixel, 1/2.2f);
}

void clamp(math::vec3f& pixel)
{
  pixel = math::clamp(pixel, 0.0f, 1.0f);
}

void toneMap(math::vec3f& newPixel, const math::vec3f& oldPixel, const grade_params& params)
{
  exposure(newPixel, oldPixel, params.exposure);
  if (params.filmic) filmic(newPixel);
  if (params.srgb) srgb(newPixel);
  clamp(newPixel);
}

void colorTint(math::vec3f& pixel, const math::vec3f& tint)
{
  pixel *= tint;
}

void saturation(math::vec3f& pixel, float saturation)
{
  float g = (pixel.x + pixel.y + pixel.z) / 3;
  pixel = g + ( pixel - g ) * (saturation  * 2);
}

void contrast(math::vec3f& pixel, float contrast)
{
  pixel = gain(pixel, 1 - contrast);
}

void vignette(math::vec3f& pixel, float vignette, float r)
{
  float vr = 1 - vignette;
  float vValue = 1 - smoothstep(vr, 2*vr, r);
  pixel *= vValue;
}

void grain(math::vec3f& pixel, float grain, math::rng_state& rng)
{
  //std::cout << (rand1f(rng) - 0.5f) << std::endl;
  pixel += (rand1f(rng) - 0.5f) * grain;
}

void mosaic(math::vec3f& pixel, math::vec3f& newPixel)
{
  pixel.x = newPixel.x;
  pixel.y = newPixel.y;
  pixel.z = newPixel.z;
}

img::image<vec4f> grade_image(
  const img::image<vec4f>& img, const grade_params& params) 
  {
    const vec2i size = img.size();
    img::image<vec4f> imgCopy(size);
    const math::vec2f fSize(size.x*1.0f, size.y*1.0f);
    math::rng_state rng = make_rng(0);
    for(int r = 0; r < size.y; r++)
      for(int c = 0; c < size.x; c++)
        {
          // ricavo il pixel in posizione [row*width+col]
          math::vec3f& newPixel = xyz(imgCopy[{c, r}]);
          const math::vec3f& oldPixel = xyz(img[{c, r}]);
          // applico il tonemap
          toneMap(newPixel, oldPixel, params);
          // applico la color tint
          colorTint(newPixel, params.tint);
          // saturation
          saturation(newPixel, params.saturation);
          // applico il contrast
          contrast(newPixel, params.contrast);
          // vignette
          vignette(newPixel, params.vignette, length({c - (fSize/2.0f).x, r - (fSize/2.0f).y}) / length(fSize/2.0f));
          // grain
          grain(newPixel, params.grain, rng);
          // mosaic
          if(params.mosaic != 0) mosaic(newPixel, xyz(imgCopy[{c - c % params.mosaic, r - r % params.mosaic}]));
        }
    if(params.grid != 0) 
    {
      for(int r = 0; r < size.y; r++)
        for(int c = 0; c < size.x; c++)
        {
          // grid
          xyz(imgCopy[r*size.x+c]) *= (c % params.grid == 0 || r % params.grid == 0 ) ? 0.5f : 1.0f; 
        }
    }
    return imgCopy;
  }



}  // namespace yocto::grade