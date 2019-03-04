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

#include "surfel_next_best_view_render_robot_urdf_plugin.h"

#include "shaders/surfel_vertex_shader_robot.h"
#include "shaders/surfel_fragment_shader_robot.h"

#include <surfel_next_best_view/surfel_renderer.h>

#include <pluginlib/class_list_macros.h>

//Realtime URDF Filter
#include "urdf_filter.h"
#include "urdf_renderer.h"

RenderRobotURDFPlugin::RenderRobotURDFPlugin()
{
  m_parent_renderer = NULL;

  m_program_robot_id = 0;
  m_vertex_shader_robot_id = 0;
  m_fragment_shader_robot_id = 0;
}

void RenderRobotURDFPlugin::setSurfelRenderer(SurfelRenderer * parent_renderer)
{
  if (m_fragment_shader_robot_id)
    glDeleteShader(m_fragment_shader_robot_id);
  if (m_vertex_shader_robot_id)
    glDeleteShader(m_vertex_shader_robot_id);
  if (m_program_robot_id)
    glDeleteProgram(m_program_robot_id);

  m_parent_renderer = parent_renderer;
  m_with_color = m_parent_renderer->GetWithColor();
  m_size = m_parent_renderer->GetSize();

  m_realtimeURDFFilter.reset(new
                       realtime_urdf_filter::RealtimeURDFFilter(m_parent_renderer->GetNodeHandle(), m_parent_renderer->GetSize()));

  //SHADER ROBOT
  m_program_robot_id = glCreateProgram();
  m_parent_renderer->CheckOpenGLError("create robot program");

  m_vertex_shader_robot_id = glCreateShader(GL_VERTEX_SHADER);
  m_parent_renderer->CheckOpenGLError("create robot vertex shader");
  const char * rvsource = VERTEX_SHADER_ROBOT.c_str();
  glShaderSource(m_vertex_shader_robot_id, 1, &rvsource, NULL);
  glCompileShader(m_vertex_shader_robot_id);
  m_parent_renderer->CheckShaderError(m_vertex_shader_robot_id,"robot vertex shader");
  glAttachShader(m_program_robot_id, m_vertex_shader_robot_id);
  m_parent_renderer->CheckOpenGLError("attach robot vertex shader");

  m_fragment_shader_robot_id = glCreateShader(GL_FRAGMENT_SHADER);
  m_parent_renderer->CheckOpenGLError("create robot fragment shader");
  const std::string rfsourcestr = GetRobotFragmentShaderCode(m_with_color);
  const char* rfsource = rfsourcestr.c_str();
  glShaderSource(m_fragment_shader_robot_id, 1, &rfsource, NULL);
  glCompileShader(m_fragment_shader_robot_id);
  m_parent_renderer->CheckShaderError(m_fragment_shader_robot_id,"robot fragment shader");
  glAttachShader(m_program_robot_id, m_fragment_shader_robot_id);
  m_parent_renderer->CheckOpenGLError("attach robot fragment shader");

  glLinkProgram(m_program_robot_id);
  m_parent_renderer->CheckLinkError(m_program_robot_id,"render");

  m_maxdepth_robot_location = glGetUniformLocation(m_program_robot_id,"maxDepth");
  m_cam_robot_location = glGetUniformLocation(m_program_robot_id, "cam");
  m_t_inv_robot_location = glGetUniformLocation(m_program_robot_id, "t_inv");
  m_depth_distance_threshold_robot_location = glGetUniformLocation(m_program_robot_id, "depth_distance_threshold");
  m_draw_robot_depth_robot_location = glGetUniformLocation(m_program_robot_id, "draw_robot_depth");
  m_image_size_robot_location = glGetUniformLocation(m_program_robot_id, "image_size");
  m_parent_renderer->CheckOpenGLError("get uniform location robot");
}

RenderRobotURDFPlugin::~RenderRobotURDFPlugin()
{
  glDeleteShader(m_fragment_shader_robot_id);
  glDeleteShader(m_vertex_shader_robot_id);
  glDeleteProgram(m_program_robot_id);
}

void RenderRobotURDFPlugin::Render(const Eigen::Affine3f & pose,
                             const sensor_msgs::JointState * joint_states,
                             const Eigen::Affine3d & robot_position)
{
  if (!m_parent_renderer)
  {
    ROS_ERROR("surfel_next_best_view: RenderRobotPlugin: plugin not initialized!");
    return;
  }

  if (!joint_states)
    return;

  const Eigen::Affine3f pose_inv = pose.inverse();
  const Eigen::Matrix4f t_inv = pose_inv.matrix();
  const Eigen::Vector2f range = m_parent_renderer->GetRange();
  const Eigen::Vector2f center = m_parent_renderer->GetCenter();
  const Eigen::Vector2f focal = m_parent_renderer->GetFocal();
  const Eigen::Vector4f cam(center.x(),center.y(),focal.x(),focal.y());

  glUseProgram(m_program_robot_id);

  glUniform1f(m_maxdepth_robot_location, GLfloat(range.y()));
  glUniform1f(m_depth_distance_threshold_robot_location, GLfloat(m_realtimeURDFFilter->getDepthDistanceThreshold()));
  glUniformMatrix4fv(m_t_inv_robot_location, 1, GL_FALSE, t_inv.data());
  glUniform4fv(m_cam_robot_location, 1, cam.data());
  glUniform2i(m_image_size_robot_location, m_size.x(), m_size.y());
  glUniform1i(m_draw_robot_depth_robot_location, GLint(m_realtimeURDFFilter->getDrawRobotDepth()));
  m_parent_renderer->CheckOpenGLError("set uniform in robot program");
  m_realtimeURDFFilter->renderRobot(*joint_states,robot_position);
  m_parent_renderer->CheckOpenGLError("render robot");

  glUseProgram(0);
}

PLUGINLIB_EXPORT_CLASS(RenderRobotURDFPlugin, RenderPlugin)
