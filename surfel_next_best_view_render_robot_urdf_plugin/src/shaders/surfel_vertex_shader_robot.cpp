/*
 * Copyright (c) 2018, Francesco Periti, Riccardo Monica
 *   RIMLab, Department of Engineering and Architecture, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "surfel_vertex_shader_robot.h"

const std::string VERTEX_SHADER_ROBOT = R"763d539980(

#version 330 compatibility

uniform mat4 t_inv; 
uniform float maxDepth;
uniform vec4 cam;
uniform ivec2 image_size;

out vec4 position;
flat out vec3 normal;
out float delete_me;

vec3 projectPoint(vec3 p)
{
  return vec3(((((cam.z * p.x) / p.z) + cam.x) - (float(image_size.x) * 0.5)) / (float(image_size.x) * 0.5),
              ((((cam.w * p.y) / p.z) + cam.y) - (float(image_size.y) * 0.5)) / (float(image_size.y) * 0.5),
              p.z / maxDepth);
}

void main(){
  position = t_inv * gl_ModelViewMatrix * gl_Vertex;

  gl_Position = vec4(projectPoint(position.xyz), 1.0);
  delete_me = 0.0;
  
  if (position.z < 0.1)
  {
    gl_Position = vec4(0.0,0.0,0.0,1.0);
    delete_me = 1.0;
  }
  
  normal =  normalize(mat3(t_inv) * gl_NormalMatrix * gl_Normal);  
}
                                                )763d539980";



