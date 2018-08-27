/*
 * Copyright (c) 2018, Riccardo Monica
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

#include "surfel_vertex_shader.h"

const std::string VERTEX_SHADER = R"763d539980(

#version 330 core

layout (location = 0) in vec4 aPositionIsFrontel;
layout (location = 1) in vec4 aNormalRadius;
layout (location = 2) in vec4 aColor;

uniform mat4 t_inv;
uniform vec4 intrinsics; //cx, cy, fx, fy
uniform float cols;
uniform float rows;
uniform float maxDepth;

out vec4 vPositionIsFrontel;
out vec4 vNormalRadius;
out vec4 vColor;
out vec3 vGlobalPos;
out float vSqrPointSize;

vec3 projectPoint(vec3 p)
{
   return vec3(((((intrinsics.z * p.x) / p.z) + intrinsics.x) - (cols * 0.5)) / (cols * 0.5),
               ((((intrinsics.w * p.y) / p.z) + intrinsics.y) - (rows * 0.5)) / (rows * 0.5),
               p.z / maxDepth);
}

void main()
{
   vec4 view_coords = t_inv * vec4(aPositionIsFrontel.xyz, 1.0);

   if(view_coords.z > maxDepth ||
      view_coords.z < 0.1)
   {
     gl_Position = vec4(1000.0f, 1000.0f, 1000.0f, 1000.0f);
     gl_PointSize = 0;
     return;
   }

   vec3 normal = mat3(t_inv) * aNormalRadius.xyz;

   if (dot(normal,view_coords.xyz) > 0.0f)
   {
     gl_Position = vec4(1000.0f, 1000.0f, 1000.0f, 1000.0f);
     gl_PointSize = 0;
     return;
   }

   gl_Position = vec4(projectPoint(view_coords.xyz), 1.0);

   vGlobalPos = aPositionIsFrontel.xyz;
   vColor = aColor;
   vNormalRadius = vec4(normal,aNormalRadius.w * aNormalRadius.w);
   vPositionIsFrontel = vec4(view_coords.xyz, aPositionIsFrontel.w);

   float pointsize = aNormalRadius.w / view_coords.z * max(intrinsics.z,intrinsics.w) * 2.0;
   vSqrPointSize = pointsize * pointsize;
   gl_PointSize = pointsize;
}

                                           )763d539980";
