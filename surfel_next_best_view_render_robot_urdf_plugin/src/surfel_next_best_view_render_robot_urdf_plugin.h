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

#ifndef ROBOT_URDF_SURFEL_RENDER_PLUGIN_H
#define ROBOT_URDF_SURFEL_RENDER_PLUGIN_H

#include <GL/glew.h>

#include <Eigen/Dense>

#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

#include <memory>
#include <string>

#include <surfel_next_best_view/surfel_next_best_view_render_plugin.h>

namespace realtime_urdf_filter {
  class RealtimeURDFFilter;
}

class RenderRobotURDFPlugin: public RenderPlugin
{
  public:
  RenderRobotURDFPlugin();

  void setSurfelRenderer(SurfelRenderer * parent_renderer);
  ~RenderRobotURDFPlugin();

  void Render(const Eigen::Affine3f & pose,
              const sensor_msgs::JointState * joint_states,
              const Eigen::Affine3d & robot_position);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  private:
  SurfelRenderer * m_parent_renderer;

  std::shared_ptr<realtime_urdf_filter::RealtimeURDFFilter> m_realtimeURDFFilter;

  bool m_with_color;
  Eigen::Vector2i m_size;

  GLint m_depth_distance_threshold_robot_location;
  GLint m_cam_robot_location;
  GLint m_t_inv_robot_location;
  GLint m_maxdepth_robot_location;
  GLint m_image_size_robot_location;
  GLint m_draw_robot_depth_robot_location;

  GLint m_program_robot_id;
  GLhandleARB m_vertex_shader_robot_id;
  GLhandleARB m_fragment_shader_robot_id;
};

#endif // ROBOT_URDF_SURFEL_RENDER_PLUGIN_H
