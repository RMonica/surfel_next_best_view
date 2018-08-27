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

#include <surfel_next_best_view/surfel_renderer.h>

#include "shaders/surfel_vertex_shader.h"
#include "shaders/surfel_fragment_shader.h"
#include <map>
#include <string>
#include <stdint.h>

#include <surfel_next_best_view/surfel_next_best_view_render_plugin.h>

template <typename T>
  T SQR(const T & t) {return t * t; }

SurfelRenderer::SurfelRenderer(ros::NodeHandle &nh,
                               const Eigen::Vector2i & size,
                               const Eigen::Vector2f & center,
                               const Eigen::Vector2f & focal,
                               const Eigen::Vector2f & range,
                               const bool with_color,
                               const bool with_depth,
                               const bool enable_lighting,
                               const RendererFilterData & filter_data,
                               RenderPlugin * render_plugin):
  m_size(size),
  m_center(center),
  m_focal(focal),
  m_range(range),
  m_with_color(with_color),
  m_with_depth(with_depth),
  m_enable_lighting(enable_lighting),
  m_nh(nh),
  m_filter_data(filter_data),
  m_render_plugin(render_plugin)
{
  if (m_render_plugin)
    m_render_plugin->setSurfelRenderer(this);

  // VIRTUAL VIEWPORT
  // generate frame buffer
  glGenFramebuffers(1,&m_framebuffer_id);
  CheckOpenGLError("glGenFramebuffers");

  // generate render buffer
  glGenRenderbuffersEXT(1,&m_renderbuffer_id);
  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT,m_renderbuffer_id);
  glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT,GL_DEPTH_COMPONENT24,m_size.x(),m_size.y());
  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);
  CheckOpenGLError("glGenRenderbuffers");

  // attach render buffer
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,m_framebuffer_id);
  glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT,GL_DEPTH_ATTACHMENT_EXT,GL_RENDERBUFFER_EXT,m_renderbuffer_id);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);
  CheckOpenGLError("attach render buffer");

  // generate color texture
  glGenTextures(1,&m_color_texture_id);
  CheckOpenGLError("generate color texture");

  glBindTexture(GL_TEXTURE_2D,m_color_texture_id);
  if (m_with_color)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, m_size.x(), m_size.y(), 0, GL_LUMINANCE, GL_FLOAT, NULL);
  else
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, m_size.x(), m_size.y(), 0, GL_LUMINANCE, GL_FLOAT, NULL);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_2D,0);
  CheckOpenGLError("configure color texture");

  // generate depth texture
  m_depth_texture_id = 0;
  if (m_with_depth)
  {
    glGenTextures(1,&m_depth_texture_id);
    CheckOpenGLError("generate depth texture");

    glBindTexture(GL_TEXTURE_2D,m_depth_texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, m_size.x(), m_size.y(), 0, GL_LUMINANCE, GL_FLOAT, NULL);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D,0);
    CheckOpenGLError("configure depth texture");
  }

  // attach texture
  const GLenum color_attachment = GL_COLOR_ATTACHMENT0_EXT + 0;
  const GLenum depth_attachment = GL_COLOR_ATTACHMENT0_EXT + 1;
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,m_framebuffer_id);
  glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,color_attachment,GL_TEXTURE_2D,m_color_texture_id,0);
  if (m_with_depth)
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,depth_attachment,GL_TEXTURE_2D,m_depth_texture_id,0);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);
  CheckOpenGLError("attach color texture");

  GLenum drawbuffers[2] = {color_attachment,depth_attachment};
  if (!m_with_depth)
    glNamedFramebufferDrawBuffers(m_framebuffer_id,1,drawbuffers);
  else
    glNamedFramebufferDrawBuffers(m_framebuffer_id,2,drawbuffers);
  CheckOpenGLError("set drawbuffer list");

  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,m_framebuffer_id);
  const GLenum completeness = glCheckFramebufferStatus(GL_FRAMEBUFFER_EXT);
  if(completeness != GL_FRAMEBUFFER_COMPLETE)
  {
    ROS_FATAL("surfel_next_best_view: error (%d): framebuffer is incomplete!",int(completeness));
    exit(1);
  }
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);

  // SHADER
  m_program_id = glCreateProgram();
  CheckOpenGLError("create program");

  m_vertex_shader_id = glCreateShader(GL_VERTEX_SHADER);
  CheckOpenGLError("create vertex shader");
  const char * vsource = VERTEX_SHADER.c_str();
  glShaderSource(m_vertex_shader_id, 1, &vsource, NULL);
  glCompileShader(m_vertex_shader_id);
  CheckShaderError(m_vertex_shader_id,"vertex shader");
  glAttachShader(m_program_id,m_vertex_shader_id);
  CheckOpenGLError("attach vertex shader");

  m_fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER);
  CheckOpenGLError("create fragment shader");
  const std::string fsourcestr = GetFragmentShaderCode(m_with_color,
                                                       m_with_depth,
                                                       m_filter_data.with_bbox_filter,
                                                       m_filter_data.with_sphere_filter);
  const char* fsource = fsourcestr.c_str();
  glShaderSource(m_fragment_shader_id, 1, &fsource, NULL);
  glCompileShader(m_fragment_shader_id);
  CheckShaderError(m_fragment_shader_id,"fragment shader");
  glAttachShader(m_program_id,m_fragment_shader_id);
  CheckOpenGLError("attach fragment shader");

  glLinkProgram(m_program_id);
  CheckLinkError(m_program_id,"render");

  glUseProgram(m_program_id);
  m_cam_location = glGetUniformLocation(m_program_id,"intrinsics");
  m_t_inv_location = glGetUniformLocation(m_program_id,"t_inv");
  m_cols_location = glGetUniformLocation(m_program_id,"cols");
  m_rows_location = glGetUniformLocation(m_program_id,"rows");
  m_maxdepth_location = glGetUniformLocation(m_program_id,"maxDepth");
  m_mindepth_location = glGetUniformLocation(m_program_id,"minDepth");
  if (m_filter_data.with_bbox_filter)
  {
    m_bbox_filter_data_min_location = glGetUniformLocation(m_program_id,"bbox_filter_data_min");
    m_bbox_filter_data_max_location = glGetUniformLocation(m_program_id,"bbox_filter_data_max");
  }
  if (m_filter_data.with_sphere_filter)
  {
    m_sphere_filter_data_location = glGetUniformLocation(m_program_id,"sphere_filter_data");
  }
  m_enable_lighting_location = glGetUniformLocation(m_program_id,"enable_lighting");
  CheckOpenGLError("get uniform location");
}

SurfelRenderer::~SurfelRenderer()
{
  glDeleteFramebuffersEXT(1,&m_framebuffer_id);
  glDeleteRenderbuffersEXT(1,&m_renderbuffer_id);
  glDeleteTextures(1,&m_color_texture_id);
  if (m_depth_texture_id)
    glDeleteTextures(1,&m_depth_texture_id);

  glDeleteShader(m_fragment_shader_id);
  glDeleteShader(m_vertex_shader_id);
  glDeleteProgram(m_program_id);
}

SurfelRenderer::GLFloatVector SurfelRenderer::Render(const GPUSurfelCloud & cloud,
                                                     const Eigen::Affine3f & pose,
                                                     const sensor_msgs::JointState * joint_states,
                                                     const Eigen::Affine3d & robot_position)
{
  ros::Time render_start_time = ros::Time::now();

  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  glBindFramebuffer(GL_FRAMEBUFFER_EXT,m_framebuffer_id);

  glClearColor(0.0,0.0,0.0,0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPushAttrib(GL_VIEWPORT_BIT);

  glViewport(0,0,m_size.x(),m_size.y());

  glUseProgram(m_program_id);

  const Eigen::Affine3f pose_inv = pose.inverse();
  const Eigen::Matrix4f t_inv = pose_inv.matrix();
  const Eigen::Vector4f cam(m_center.x(),m_center.y(),m_focal.x(),m_focal.y());

  glUniform4fv(m_cam_location,1,cam.data());
  glUniformMatrix4fv(m_t_inv_location,1,GL_FALSE,t_inv.data());
  glUniform1f(m_cols_location,GLfloat(m_size.x()));
  glUniform1f(m_rows_location,GLfloat(m_size.y()));
  glUniform1f(m_maxdepth_location,GLfloat(m_range.y()));
  glUniform1f(m_mindepth_location,GLfloat(m_range.x()));
  glUniform1i(m_enable_lighting_location,GLint(m_enable_lighting ? 1 : 0));

  if (m_filter_data.with_bbox_filter)
  {
    glUniform3fv(m_bbox_filter_data_min_location,1,m_filter_data.bbox_filter_min.data());
    glUniform3fv(m_bbox_filter_data_max_location,1,m_filter_data.bbox_filter_max.data());
  }

  if (m_filter_data.with_sphere_filter)
  {
    const Eigen::Vector3f local_center = pose_inv * m_filter_data.sphere_filter_center;
    const GLfloat data[4] = {local_center.x(),local_center.y(),local_center.z(),
                             m_filter_data.sphere_filter_radius
                            };
    glUniform4fv(m_sphere_filter_data_location,1,data);
  }

  if (!cloud.IsEmpty()) // prevent crash
  {
    glBindBuffer(GL_ARRAY_BUFFER,cloud.GetId());

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(0,4,GL_FLOAT,GL_FALSE,cloud.GetVertexSize() * sizeof(GLfloat),(GLvoid *)(0 * sizeof(GLfloat) * 4));
    glVertexAttribPointer(1,4,GL_FLOAT,GL_FALSE,cloud.GetVertexSize() * sizeof(GLfloat),(GLvoid *)(1 * sizeof(GLfloat) * 4));

    if (m_with_color)
    {
      glBindBuffer(GL_ARRAY_BUFFER,cloud.GetColorId());
      glEnableVertexAttribArray(2);
      glVertexAttribPointer(2,4,GL_FLOAT,GL_FALSE,cloud.GetColorSize() * sizeof(GLfloat),(GLvoid *)(0 * sizeof(GLfloat) * 4));
    }

    glDrawArrays(GL_POINTS,0,cloud.GetSize());

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    if (m_with_color)
      glDisableVertexAttribArray(2);
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);
  }

  glPopAttrib();

  if (joint_states && m_render_plugin)
  {
    glUseProgram(0);
    m_render_plugin->Render(pose, joint_states, robot_position);
    glUseProgram(m_program_id);
  }

  //glBindFramebuffer(GL_FRAMEBUFFER_EXT,0);

  //glDisable(GL_PROGRAM_POINT_SIZE);
  //glDisable(GL_DEPTH_TEST);

  glFinish();

  CheckOpenGLError("render");

  m_rendering_time = ros::Time::now() - render_start_time;
  ros::Time download_start_time = ros::Time::now();

  GLFloatVector result;
  if (m_with_color)
  {
    result.resize(m_size.x() * m_size.y() * 4);
    glBindTexture(GL_TEXTURE_2D,m_color_texture_id);
    glGetTexImage(GL_TEXTURE_2D,0,GL_RGBA,GL_FLOAT,(GLvoid *)(result.data()));
    glBindTexture(GL_TEXTURE_2D,0);
  }
  else
  {
    result.resize(m_size.x() * m_size.y());
    glBindTexture(GL_TEXTURE_2D,m_color_texture_id);
    glGetTexImage(GL_TEXTURE_2D,0,GL_RED,GL_FLOAT,(GLvoid *)(result.data()));
    glBindTexture(GL_TEXTURE_2D,0);
  }
  CheckOpenGLError("download color texture");

  if (m_with_depth)
  {
    m_depth_result.resize(m_size.x() * m_size.y());
    glBindTexture(GL_TEXTURE_2D,m_depth_texture_id);
    glGetTexImage(GL_TEXTURE_2D,0,GL_RED,GL_FLOAT,(GLvoid *)(m_depth_result.data()));
    glBindTexture(GL_TEXTURE_2D,0);
  }
  CheckOpenGLError("download depth texture");

  {
    const ros::Time now = ros::Time::now();
    m_download_time = now - download_start_time;
    m_run_time = now - render_start_time;
  }

  return result;
}

GPUSurfelCloud::GPUSurfelCloud(const PointSurfelCloud &cloud,const RendererFilterData & input_filter)
{
  m_input_filter = input_filter;

  const size_t cloud_size = cloud.size();
  const size_t vertex_size = GetVertexSize();
  const size_t color_size = GetColorSize();
  std::vector<GLfloat> data(cloud_size * vertex_size);
  std::vector<GLfloat> data_color(cloud_size * color_size);

  size_t counter_out = 0;
  for (size_t i = 0; i < cloud_size; i++)
  {
    const pcl::PointSurfel & pt = cloud[i];
    if (m_input_filter.with_bbox_filter)
    {
      const Eigen::Vector3f ept(pt.x,pt.y,pt.z);
      if ((ept.array() > m_input_filter.bbox_filter_max.array()).any() ||
          (ept.array() < m_input_filter.bbox_filter_min.array()).any())
        continue;
    }

    if (m_input_filter.with_sphere_filter)
    {
      const Eigen::Vector3f ept(pt.x,pt.y,pt.z);
      if ((ept - m_input_filter.sphere_filter_center).squaredNorm() > SQR(m_input_filter.sphere_filter_radius))
        continue;
    }

    float * const first = data.data() + counter_out * vertex_size;
    size_t c = 0;
    first[c++] = pt.x;
    first[c++] = pt.y;
    first[c++] = pt.z;
    first[c++] = pt.confidence;

    first[c++] = pt.normal_x;
    first[c++] = pt.normal_y;
    first[c++] = pt.normal_z;
    first[c++] = pt.radius;

    float * const first_color = data_color.data() + counter_out * color_size;
    c = 0;
    first_color[c++] = pt.r / 255.0;
    first_color[c++] = pt.g / 255.0;
    first_color[c++] = pt.b / 255.0;
    first_color[c++] = 0.0;

    counter_out++;
  }

  data.resize(counter_out * vertex_size);
  data_color.resize(counter_out * color_size);
  m_size = counter_out;

  glGenBuffers(1,&m_buffer_id);
  glBindBuffer(GL_ARRAY_BUFFER,m_buffer_id);
  if (m_size != 0) // prevent crash
    glBufferStorage(GL_ARRAY_BUFFER,data.size() * sizeof(GLfloat),(GLvoid *)(data.data()),0);

  glGenBuffers(1,&m_color_buffer_id);
  glBindBuffer(GL_ARRAY_BUFFER,m_color_buffer_id);
  if (m_size != 0) // prevent crash
    glBufferStorage(GL_ARRAY_BUFFER,data_color.size() * sizeof(GLfloat),(GLvoid *)(data_color.data()),0);

  glBindBuffer(GL_ARRAY_BUFFER,0);
}

GPUSurfelCloud::~GPUSurfelCloud()
{
  glDeleteBuffers(1,&m_buffer_id);
  glDeleteBuffers(1,&m_color_buffer_id);
}

void SurfelRenderer::CheckOpenGLError(const std::string message)
{
  static std::map<GLenum,std::string> errors;
  if (errors.empty())
  {
#define SurfelRenderer_CheckOpenGLError_ERROR_MACRO(a) \
        errors.insert(std::pair<GLenum,std::string>((a),(#a)));
    SurfelRenderer_CheckOpenGLError_ERROR_MACRO(GL_NO_ERROR);
    SurfelRenderer_CheckOpenGLError_ERROR_MACRO(GL_INVALID_ENUM);
    SurfelRenderer_CheckOpenGLError_ERROR_MACRO(GL_INVALID_VALUE);
    SurfelRenderer_CheckOpenGLError_ERROR_MACRO(GL_INVALID_OPERATION);
    SurfelRenderer_CheckOpenGLError_ERROR_MACRO(GL_STACK_OVERFLOW);
    SurfelRenderer_CheckOpenGLError_ERROR_MACRO(GL_STACK_UNDERFLOW);
    SurfelRenderer_CheckOpenGLError_ERROR_MACRO(GL_OUT_OF_MEMORY);
    SurfelRenderer_CheckOpenGLError_ERROR_MACRO(GL_TABLE_TOO_LARGE);
#undef SurfelRenderer_CheckOpenGLError_ERROR_MACRO
  }

  GLenum error = glGetError();

  if (error != GL_NO_ERROR)
  {
    auto iter = errors.find(error);
    if (iter != errors.end())
    {
      const char * es = iter->second.c_str();
      ROS_FATAL("surfel_next_best_view: openGL error: %s (%d)\nmessage: %s",es,int(error),message.c_str());
    }
    else
    {
      ROS_FATAL("surfel_next_best_view: openGL error: (unknown error) (%d)\nphase: %s",int(error),message.c_str());
    }
    exit(1);
  }
}

void SurfelRenderer::CheckShaderError(const GLhandleARB shader_id,const std::string message)
{
  GLint status;
  glGetShaderiv(shader_id, GL_COMPILE_STATUS, &status);
  if(status != GL_TRUE) {
    const int SHADER_LOG_MAX_LEN = 10240;
    std::vector<char> infolog(SHADER_LOG_MAX_LEN + 1);
    GLsizei len;
    glGetShaderInfoLog(shader_id, SHADER_LOG_MAX_LEN, &len, infolog.data());
    infolog[len + 1] = '\0';
    ROS_FATAL("surfel_next_best_view: GLSL %s Shader compilation failed:\n%s\n\n",message.c_str(),infolog.data());
    exit(1);
  }
}

void SurfelRenderer::CheckLinkError(const GLint program_id, const std::string message)
{
  GLint status;
  glGetProgramiv(program_id, GL_LINK_STATUS, &status);
  if(status != GL_TRUE) {
    const int PROGRAM_LOG_MAX_LEN = 10240;
    std::vector<char> infolog(PROGRAM_LOG_MAX_LEN + 1);
    GLsizei len;
    glGetProgramInfoLog(program_id, PROGRAM_LOG_MAX_LEN, &len, infolog.data());
    infolog[len + 1] = '\0';
    ROS_FATAL("surfel_next_best_view: GLSL %s Program link failed:\n%s\n\n",message.c_str(),infolog.data());
    exit(1);
  }
}

bool RendererFilterData::operator==(const RendererFilterData & other) const
{
  if (with_bbox_filter)
  {
    if (bbox_filter_max != other.bbox_filter_max)
      return false;
    if (bbox_filter_min != other.bbox_filter_min)
      return false;
  }
  if (with_sphere_filter)
  {
    if (sphere_filter_center != other.sphere_filter_center)
      return false;
    if (sphere_filter_radius != other.sphere_filter_radius)
      return false;
  }
  return true;
}
