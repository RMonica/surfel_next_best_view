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

#ifndef SURFEL_RENDERER_H
#define SURFEL_RENDERER_H

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <pluginlib/class_loader.h>

// STL
#include <stdint.h>
#include <vector>

// Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>

// OpenGL
#include <GL/glew.h>

// Boost
#include <boost/shared_ptr.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct RendererFilterData
{
  bool with_bbox_filter;
  Eigen::Vector3f bbox_filter_min;
  Eigen::Vector3f bbox_filter_max;
  bool with_sphere_filter;
  Eigen::Vector3f sphere_filter_center;
  float sphere_filter_radius;

  RendererFilterData()
  {
    with_bbox_filter = false;
    with_sphere_filter = false;
  }

  bool operator==(const RendererFilterData & other) const;
  bool operator!=(const RendererFilterData & other) const {return !((*this) == other); }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class GPUSurfelCloud
{
  public:
  typedef pcl::PointCloud<pcl::PointSurfel> PointSurfelCloud;
  GPUSurfelCloud(const PointSurfelCloud & cloud,const RendererFilterData & input_filter);
  ~GPUSurfelCloud();

  GLuint GetId() const {return m_buffer_id; }
  GLuint GetColorId() const {return m_color_buffer_id; }
  size_t GetVertexSize() const {return 8; }
  size_t GetColorSize() const {return 4; }
  size_t GetSize() const {return m_size; }
  bool IsEmpty() const {return GetSize() == 0; }

  const RendererFilterData & GetInputFilter() const {return m_input_filter; }

  typedef boost::shared_ptr<GPUSurfelCloud> Ptr;
  typedef boost::shared_ptr<GPUSurfelCloud> ConstPtr;

  private:
  GLuint m_buffer_id;
  GLuint m_color_buffer_id;
  size_t m_size;

  RendererFilterData m_input_filter;

  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class RenderPlugin;

class SurfelRenderer
{
  public:
  typedef uint64_t uint64;
  typedef uint8_t uint8;
  typedef pcl::PointCloud<pcl::PointSurfel> PointSurfelCloud;
  typedef std::vector<GLfloat> GLFloatVector;

  struct RGBA
  {
    GLfloat r,g,b,a;

    RGBA(): r(0),g(0),b(0),a(0) {}
    RGBA(const float rr,const float gg,const float bb,const float aa): r(rr), g(gg), b(bb), a(aa) {}
  } __attribute__((packed));

  typedef std::vector<RGBA> RGBAVector;

  typedef SurfelRenderer Self;
  typedef boost::shared_ptr<Self> Ptr;
  typedef boost::shared_ptr<const Self> ConstPtr;

  SurfelRenderer(ros::NodeHandle &nodeHandle, const Eigen::Vector2i & size, const Eigen::Vector2f & center,
                 const Eigen::Vector2f & focal, const Eigen::Vector2f & range,
                 const bool with_color, const bool with_depth,
                 const bool enable_lighting, const RendererFilterData & filter_data,
                 RenderPlugin * render_plugin);
  ~SurfelRenderer();

  GLFloatVector Render(const GPUSurfelCloud & cloud, const Eigen::Affine3f & pose,
                       const sensor_msgs::JointState * joint_states, const Eigen::Affine3d & robot_position);

  const Eigen::Vector2i & GetSize() {return m_size; }
  const Eigen::Vector2f & GetCenter() {return m_center; }
  const Eigen::Vector2f & GetFocal() {return m_focal; }
  const Eigen::Vector2f & GetRange() {return m_range; }
  const bool GetWithColor() {return m_with_color; }
  const bool GetWithDepth() {return m_with_depth; }
  const bool GetEnableLighting() {return m_enable_lighting; }

  const RendererFilterData & GetFilterData() {return m_filter_data; }

  void CheckOpenGLError(const std::string message);
  void CheckShaderError(const GLhandleARB shader_id,const std::string message);
  void CheckLinkError(const GLint program_id, const std::string message);

  const ros::Duration & GetLastRenderingTime() const {return m_rendering_time; }
  const ros::Duration & GetLastDownloadTime() const {return m_download_time; }
  const ros::Duration & GetLastRunTime() const {return m_run_time; }

  const GLFloatVector & GetLastDepthResult() const {return m_depth_result; }

  ros::NodeHandle & GetNodeHandle() const {return m_nh; }

  private:
  Eigen::Vector2i m_size;
  Eigen::Vector2f m_center;
  Eigen::Vector2f m_focal;
  Eigen::Vector2f m_range; // min, max
  bool m_with_color;
  bool m_with_depth;
  bool m_enable_lighting;

  ros::NodeHandle & m_nh;

  RendererFilterData m_filter_data;
  GLuint m_renderbuffer_id;
  GLuint m_framebuffer_id;
  GLuint m_color_texture_id;
  GLuint m_depth_texture_id;

  GLFloatVector m_depth_result;

  GLint m_cam_location;
  GLint m_t_inv_location;
  GLint m_cols_location;
  GLint m_rows_location;
  GLint m_maxdepth_location;
  GLint m_mindepth_location;
  GLint m_bbox_filter_data_min_location;
  GLint m_bbox_filter_data_max_location;
  GLint m_sphere_filter_data_location;
  GLint m_enable_lighting_location;

  GLint m_program_id;
  GLhandleARB m_vertex_shader_id;
  GLhandleARB m_fragment_shader_id;
  
  ros::Duration m_rendering_time;
  ros::Duration m_download_time;
  ros::Duration m_run_time;

  RenderPlugin * m_render_plugin;

  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif // SURFEL_RENDERER_H
