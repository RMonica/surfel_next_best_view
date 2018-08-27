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

#include "surfel_next_best_view_node.h"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <actionlib/server/simple_action_server.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// STL
#include <stdint.h>
#include <vector>
#include <string>

// Boost
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>

// custom
#include <surfel_next_best_view_msgs/EvaluatePosesAction.h>
#include <surfel_next_best_view_msgs/SetPointCloudAction.h>
#include <init_fake_opengl_context/fake_opengl_context.h>
#include <surfel_next_best_view/surfel_renderer.h>
#include <surfel_next_best_view/surfel_next_best_view_render_plugin.h>

template <typename T> T SQR(const T n) {return n*n; }

struct PointXYZIRGBA
{
  PCL_ADD_POINT4D;
  union
  {
    struct
    {
      PCL_ADD_UNION_RGB;
      float intensity;
    };
    uint32_t intensity_data[4];
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRGBA,
                                  (float,x,x)
                                  (float,y,y)
                                  (float,z,z)
                                  (uint32_t,rgba,rgba)
                                  (float,intensity,intensity)
)

class SurfelNextBestView
{
  public:
  typedef uint64_t uint64;
  typedef uint32_t uint32;
  typedef uint8_t uint8;
  typedef int64_t int64;
  typedef std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > Affine3fVector;
  typedef pcl::PointCloud<pcl::PointSurfel> PointSurfelCloud;
  typedef pcl::PointCloud<pcl::PointXYZI> PointXYZICloud;
  typedef pcl::PointCloud<PointXYZIRGBA> PointXYZRGBICloud;
  typedef actionlib::SimpleActionServer<surfel_next_best_view_msgs::EvaluatePosesAction> EvaluatePosesActionServer;
  typedef actionlib::SimpleActionServer<surfel_next_best_view_msgs::SetPointCloudAction> SetPointCloudActionServer;

  SurfelNextBestView(ros::NodeHandle & nh): m_nh(nh)
  {
    boost::mutex::scoped_lock lock(m_mutex);
    std::string param_string;

    m_opengl_context_initialized = false;
    m_is_new_cloud = false;
    m_is_processing = false;

    m_nh.param<std::string>(PARAM_NAME_SET_POINT_CLOUD_TOPIC,param_string,PARAM_DEFAULT_SET_POINT_CLOUD_TOPIC);
    m_point_cloud_sub = m_nh.subscribe(param_string,1,&SurfelNextBestView::onPointCloud,this);

    m_nh.param<std::string>(PARAM_NAME_FAKE_OPENGL_CONTEXT_SCREEN,m_opengl_context_screen,
                            PARAM_DEFAULT_FAKE_OPENGL_CONTEXT_SCREEN);

    m_nh.param<std::string>(PARAM_NAME_EVALUATE_POSES_ACTION,param_string,PARAM_DEFAULT_EVALUATE_POSES_ACTION);
    m_poses_as.reset(new EvaluatePosesActionServer
                     (m_nh,param_string, boost::bind(&SurfelNextBestView::evaluatePoses, this, _1), false));
    m_nh.param<std::string>(PARAM_NAME_SET_POINT_CLOUD_ACTION,param_string,PARAM_DEFAULT_SET_POINT_CLOUD_ACTION);
    m_cloud_as.reset(new SetPointCloudActionServer
                     (m_nh,param_string, boost::bind(&SurfelNextBestView::setPointCloud, this, _1), false));

    m_nh.param<std::string>(PARAM_NAME_FRAME_ID,m_frame_id,PARAM_DEFAULT_FRAME_ID);

    m_nh.param<bool>(PARAM_NAME_ENABLE_ROBOT_FILTER,m_enable_robot_filter,PARAM_DEFAULT_ENABLE_ROBOT_FILTER);
    m_nh.param<std::string>(PARAM_NAME_ROBOT_FILTER_PLUGIN,m_enable_robot_filter_plugin,PARAM_DEFAULT_ROBOT_FILTER_PLUGIN);
    m_robot_urdf_filter_ptr = NULL;

    if (m_enable_robot_filter)
    {
      m_render_plugin_loader.reset(new RenderPluginClassLoader("surfel_next_best_view", "RenderPlugin"));

      try
      {
        m_robot_urdf_filter = m_render_plugin_loader->createInstance(m_enable_robot_filter_plugin);
        m_robot_urdf_filter_ptr = &(*m_robot_urdf_filter);
        ROS_INFO("surfel_next_best_view: robot urdf filter plugin \"%s\" loaded.", m_enable_robot_filter_plugin.c_str());
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_ERROR("surfel_next_best_view: robot urdf filter plugin \"%s\": load failed, cause: %s",
                  m_enable_robot_filter_plugin.c_str(), ex.what());
      }
    }

    m_poses_as->start();
    m_cloud_as->start();
  }

  ~SurfelNextBestView()
  {
    boost::mutex::scoped_lock lock(m_mutex);
    ros::shutdown();
    m_cond.notify_all();
  }

  void CheckOpenGLInit()
  {
    if (!m_opengl_context_initialized)
    {
      ROS_INFO("surfel_next_best_view: initializing fake opengl context (Screen: \"%s\").",
               m_opengl_context_screen.c_str());
      InitFakeOpenGLContext(m_opengl_context_screen);
      m_opengl_context_initialized = true;
    }
  }

  void setPointCloud(const surfel_next_best_view_msgs::SetPointCloudGoalConstPtr & goal)
  {
    onPointCloud(goal->cloud);
    m_cloud_as->setSucceeded();
  }

  void onPointCloud(const sensor_msgs::PointCloud2 & cloud)
  {
    boost::mutex::scoped_lock lock(m_mutex);
    if (m_is_processing)
    {
      ROS_ERROR("surfel_next_best_view: can't set a cloud while processing!");
      return;
    }

    PointSurfelCloud::Ptr new_cloud(new PointSurfelCloud);
    pcl::fromROSMsg(cloud,*new_cloud);
    m_cloud = new_cloud;
    m_is_new_cloud = true;
    ROS_INFO("surfel_next_best_view: cloud set (%d points)",int(new_cloud->size()));
  }

  void evaluatePoses(const surfel_next_best_view_msgs::EvaluatePosesGoalConstPtr & goal)
  {
    {
      boost::mutex::scoped_lock lock(m_mutex);
      if (m_is_processing)
        return; // do not start processing twice

      m_is_processing = true;
      m_current_goal = goal;
      m_cond.notify_all();

      while (m_is_processing && !ros::isShuttingDown())
        m_cond.wait_for(lock, boost::chrono::duration<uint64, boost::milli>(500));

      if (ros::isShuttingDown())
        return;

      if (m_current_result)
      {
        m_poses_as->setSucceeded(*m_current_result);
        m_current_result.reset();
      }
      else
        m_poses_as->setAborted();
    }
  }

  template <uint64 step,uint64 offset>
  void CountImageValues(const SurfelRenderer::GLFloatVector & imgvector,
                        const uint64 image_size,
                        double & unk_counter,double & occ_counter,
                        uint64 & unk_num,uint64 & occ_num,uint64 & oor_num)
  {
    for (uint64 i = 0; i < image_size; i++)
    {
      const double gain = imgvector[i * step + offset];

      if (gain < 0.0) // unknown
      {
        unk_counter += std::abs(gain);
        unk_num++;
      }
      else if (gain > 0.0) // occupied
      {
        occ_counter += gain;
        occ_num++;
      }
      else // empty
      {
        oor_num++;
      }
    }
  }

  template <typename PointT,bool with_color>
  struct BuildCloudFromImageHelper
  {
    static void StoreColor(PointT & pt,const uint8 r,const uint8 g,const uint8 b,const uint8 a) {}
  };

  template <typename PointT>
  struct BuildCloudFromImageHelper<PointT,true>
  {
    static void StoreColor(PointT & pt,const uint8 r,const uint8 g,const uint8 b,const uint8 a)
      {pt.r = r; pt.g = g; pt.b = b; pt.a = a; }
  };

  template <typename PointT,bool with_color>
  sensor_msgs::PointCloud2 BuildCloudFromImage(const Eigen::Vector2i & new_size,const Eigen::Vector2f & new_center,
                                               const Eigen::Vector2f & new_focal,
                                               const SurfelRenderer::GLFloatVector & imgvector,
                                               const uint64 step, const SurfelRenderer::GLFloatVector & depth_result,
                                               const uint64 r_offset,const uint64 g_offset,
                                               const uint64 b_offset,const uint64 a_offset)
  {
    pcl::PointCloud<PointT> cloud;
    cloud.resize(new_size.x() * new_size.y());
    cloud.width = new_size.x();
    cloud.height = new_size.y();
    cloud.is_dense = false;

    for (int64 y = 0; y < new_size.y(); y++)
      for (int64 x = 0; x < new_size.x(); x++)
      {
        const uint64 i = x + y * new_size.x();
        PointT & pt = cloud[i];
        const float intensity = imgvector[i * step + a_offset];
        pt.intensity = intensity;

        const Eigen::Vector3f bearing = Eigen::Vector3f((x - new_center.x()) / new_focal.x(),
                                                        (y - new_center.y()) / new_focal.y(),
                                                        1.0);
        const Eigen::Vector3f ept = depth_result[i] * bearing;
        pt.x = ept.x();
        pt.y = ept.y();
        pt.z = ept.z();

        const uint8 r = imgvector[i * step + r_offset] * 255.9;
        const uint8 g = imgvector[i * step + g_offset] * 255.9;
        const uint8 b = imgvector[i * step + b_offset] * 255.9;
        const uint8 a = 255;

        BuildCloudFromImageHelper<PointT,with_color>::StoreColor(pt,r,g,b,a);
      }

    sensor_msgs::PointCloud2 result;
    pcl::toROSMsg(cloud,result);
    return result;
  }

  // any OpenGL processing must be done by the main thread (avoid driver bugs)
  void Run()
  {
    while (true)
    {
      // critical section
      {
        boost::mutex::scoped_lock lock(m_mutex);
        while (!m_current_goal && !ros::isShuttingDown())
          m_cond.wait_for(lock, boost::chrono::duration<uint64, boost::milli>(500));

        if (ros::isShuttingDown())
          return;
      }

      const surfel_next_best_view_msgs::EvaluatePosesGoalConstPtr & goal = m_current_goal;

      if (!m_cloud)
      {
        ROS_ERROR("surfel_next_best_view: action asked for evaluation, but cloud is not set!");
        {
          boost::mutex::scoped_lock(m_mutex);
          m_is_processing = false;
          m_cond.notify_all();
          m_current_goal.reset();
          m_current_result.reset();
        }
        continue;
      }

      const ros::Time start_time = ros::Time::now();

      const ros::Time check_opengl_start_time = ros::Time::now();
      CheckOpenGLInit();
      const ros::Duration check_opengl_time = ros::Time::now() - check_opengl_start_time;

      const ros::Time cloud_gen_start_time = ros::Time::now();
      RendererFilterData input_filter_data;
      {
        if (uint64(goal->input_filter_type) == uint64(goal->FILTER_TYPE_SPHERE))
        {
          if (goal->input_filter_data.size() != 4)
            ROS_ERROR("surfel_next_best_view: requested input FILTER_TYPE_SPHERE, but %d parameters provided (4 needed).",
                      int(goal->input_filter_data.size()));
          else
          {
            input_filter_data.with_sphere_filter = true;
            input_filter_data.sphere_filter_center.x() = goal->input_filter_data[0];
            input_filter_data.sphere_filter_center.y() = goal->input_filter_data[1];
            input_filter_data.sphere_filter_center.z() = goal->input_filter_data[2];
            input_filter_data.sphere_filter_radius = goal->input_filter_data[3];
          }
        }
        if (uint64(goal->input_filter_type) == uint64(goal->FILTER_TYPE_BBOX))
        {
          if (goal->input_filter_data.size() != 6)
            ROS_ERROR("surfel_next_best_view: requested input FILTER_TYPE_BBOX, but %d parameters provided (6 needed).",
                      int(goal->input_filter_data.size()));
          else
          {
            input_filter_data.with_bbox_filter = true;
            input_filter_data.bbox_filter_min.x() = goal->input_filter_data[0];
            input_filter_data.bbox_filter_min.y() = goal->input_filter_data[1];
            input_filter_data.bbox_filter_min.z() = goal->input_filter_data[2];
            input_filter_data.bbox_filter_max.x() = goal->input_filter_data[3];
            input_filter_data.bbox_filter_max.y() = goal->input_filter_data[4];
            input_filter_data.bbox_filter_max.z() = goal->input_filter_data[5];
          }
        }
      }

      if (!m_cloud_gpu || m_is_new_cloud || m_cloud_gpu->GetInputFilter() != input_filter_data)
      {
        boost::mutex::scoped_lock(m_mutex);
        ROS_INFO("surfel_next_best_view: rebuilding GPU cloud.");
        m_cloud_gpu = GPUSurfelCloud::ConstPtr(new GPUSurfelCloud(*m_cloud,input_filter_data));
        m_is_new_cloud = false;
      }
      const ros::Duration cloud_gen_time = ros::Time::now() - cloud_gen_start_time;

      Eigen::Vector2i new_size(goal->width,goal->height);
      Eigen::Vector2f new_focal(goal->focal_x,goal->focal_y);
      Eigen::Vector2f new_center(goal->center_x,goal->center_y);
      Eigen::Vector2f new_range(goal->range_min,goal->range_max);
      bool new_with_color(goal->with_color);
      bool new_with_depth(goal->with_clouds);
      bool new_enable_lighting(goal->enable_lighting);
      ROS_INFO_STREAM("surfel_next_best_view: evaluation parameters:"
                      "\n  viewport size: " << new_size.transpose() <<
                      "\n  camera focal: " << new_focal.transpose() <<
                      "\n  camera center: " << new_center.transpose() <<
                      "\n  camera range: " << new_range.transpose() <<
                      "\n  with color: " << (new_with_color ? "TRUE" : "FALSE") <<
                      "\n  with depth: " << (new_with_depth ? "TRUE" : "FALSE") <<
                      "\n  enable lighting: " << (new_enable_lighting ? "TRUE" : "FALSE") <<
                      "\n  with filter: " << goal->filter_type);

      if (goal->width == 0 || goal->height == 0 ||
          goal->focal_x <= 0.0 || goal->focal_y <= 0.0 ||
          goal->range_min == goal->range_max)
      {
        ROS_ERROR("surfel_next_best_view: invalid camera parameters!");
        {
          boost::mutex::scoped_lock(m_mutex);
          m_is_processing = false;
          m_cond.notify_all();
          m_current_goal.reset();
          m_current_result.reset();
        }
        continue;
      }

      RendererFilterData filter_data;
      {
        if (uint64(goal->filter_type) == uint64(goal->FILTER_TYPE_SPHERE))
        {
          if (goal->filter_data.size() != 4)
            ROS_ERROR("surfel_next_best_view: requested FILTER_TYPE_SPHERE, but %d parameters provided (4 needed).",
                      int(goal->filter_data.size()));
          else
          {
            filter_data.with_sphere_filter = true;
            filter_data.sphere_filter_center.x() = goal->filter_data[0];
            filter_data.sphere_filter_center.y() = goal->filter_data[1];
            filter_data.sphere_filter_center.z() = goal->filter_data[2];
            filter_data.sphere_filter_radius = goal->filter_data[3];
          }
        }
        if (uint64(goal->filter_type) == uint64(goal->FILTER_TYPE_BBOX))
        {
          if (goal->filter_data.size() != 6)
            ROS_ERROR("surfel_next_best_view: requested FILTER_TYPE_BBOX, but %d parameters provided (6 needed).",
                      int(goal->filter_data.size()));
          else
          {
            filter_data.with_bbox_filter = true;
            filter_data.bbox_filter_min.x() = goal->filter_data[0];
            filter_data.bbox_filter_min.y() = goal->filter_data[1];
            filter_data.bbox_filter_min.z() = goal->filter_data[2];
            filter_data.bbox_filter_max.x() = goal->filter_data[3];
            filter_data.bbox_filter_max.y() = goal->filter_data[4];
            filter_data.bbox_filter_max.z() = goal->filter_data[5];
          }
        }
      }

      const ros::Time generate_renderer_start_time = ros::Time::now();
      {
        if (!m_renderer)
        {
          ROS_INFO("surfel_next_best_view: generating new renderer.");
          m_renderer.reset(new SurfelRenderer(m_nh, new_size,new_center,new_focal,new_range,
                                              new_with_color,new_with_depth,new_enable_lighting,filter_data,
                                              m_robot_urdf_filter_ptr));
        }
        else
        {
          SurfelRenderer & old_renderer = *m_renderer;
          if (old_renderer.GetCenter() != new_center ||
              old_renderer.GetFocal() != new_focal ||
              old_renderer.GetSize() != new_size ||
              old_renderer.GetRange() != new_range ||
              old_renderer.GetWithColor() != new_with_color ||
              old_renderer.GetWithDepth() != new_with_depth ||
              old_renderer.GetEnableLighting() != new_enable_lighting ||
              old_renderer.GetFilterData() != filter_data)
          {
            ROS_INFO("surfel_next_best_view: replacing old renderer with new renderer.");
            m_renderer.reset(new SurfelRenderer(m_nh,new_size,new_center,new_focal,new_range,
                                                new_with_color,new_with_depth,new_enable_lighting,filter_data,
                                                m_robot_urdf_filter_ptr));
          }
        }
      }
      const ros::Duration generate_renderer_time = ros::Time::now() - generate_renderer_start_time;

      surfel_next_best_view_msgs::EvaluatePosesResultPtr result_ptr(new surfel_next_best_view_msgs::EvaluatePosesResult);
      surfel_next_best_view_msgs::EvaluatePosesResult & result = *result_ptr;

      const uint64 goal_size = goal->poses.size();
      result.scores.resize(goal_size);
      result.occupied.resize(goal_size);
      result.unknown.resize(goal_size);
      result.out_of_range.resize(goal_size);

      const bool with_images = goal->with_images;
      const bool with_clouds = goal->with_clouds;
      const float occupied_score = goal->occupied_score;
      const float unknown_score = goal->unknown_score;
      const double distance_weight = goal->distance_weight;
      const double absolute_weight = goal->absolute_weight;

      ros::Duration counting_time(0.0);
      ros::Duration rendering_time(0.0);
      ros::Duration download_time(0.0);

      ROS_INFO("surfel_next_best_view: evaluating %d poses.",int(goal_size));

      const bool with_joint_states = goal->joint_states.size() == goal_size;
      if (!with_joint_states && !goal->joint_states.empty())
        ROS_ERROR("surfel_next_best_view: joint states size does not match: it is %u, should be %u",
                  (unsigned int)(goal->joint_states.size()),(unsigned int)(goal_size));
      ROS_INFO("surfel_next_best_view: evaluation with joint states %s", (with_joint_states ? "TRUE" : "FALSE"));

      Eigen::Affine3d robot_position = Eigen::Affine3d::Identity();
      if (goal->robot_pose.orientation.x != 0.0 || goal->robot_pose.orientation.y != 0.0 || // null quaternion is default
          goal->robot_pose.orientation.z != 0.0 || goal->robot_pose.orientation.w != 0.0)   // but it's invalid
        tf::poseMsgToEigen(goal->robot_pose,robot_position);

      for (uint64 pose_i = 0; pose_i < goal_size; pose_i++)
      {
        //ROS_INFO("surfel_next_best_view: evaluating pose %d/%d",int(pose_i + 1),int(goal_size));

        if (ros::isShuttingDown())
          return;

        Eigen::Affine3d pose_d;
        tf::poseMsgToEigen(goal->poses[pose_i],pose_d);

        const sensor_msgs::JointState * const joint_state = with_joint_states ? &(goal->joint_states[pose_i]) : NULL;
        const SurfelRenderer::GLFloatVector imgvector =
          m_renderer->Render(*m_cloud_gpu,pose_d.cast<float>(),joint_state,robot_position);

        if (ros::isShuttingDown())
          return;

        rendering_time += m_renderer->GetLastRenderingTime();
        download_time += m_renderer->GetLastDownloadTime();

        ros::Time counting_start_time = ros::Time::now();
        double occ_counter = 0;
        double unk_counter = 0;
        uint64 occ_num = 0;
        uint64 unk_num = 0;
        uint64 oor_num = 0;
        const uint64 image_size = new_size.y() * new_size.x();

        if (!new_with_color)
          CountImageValues<1,0>(imgvector,image_size,
                                unk_counter,occ_counter,
                                unk_num,occ_num,oor_num);
        else
          CountImageValues<4,3>(imgvector,image_size,
                                unk_counter,occ_counter,
                                unk_num,occ_num,oor_num);
        std::cout << unk_counter << " " << unknown_score << std::endl;
        const double score = (occ_counter * occupied_score + unk_counter * unknown_score) * distance_weight +
                             (occ_num * occupied_score + unk_num * unknown_score) * absolute_weight;

        result.scores[pose_i] = score;
        result.occupied[pose_i] =     occ_counter * distance_weight + occ_num * absolute_weight;
        result.unknown[pose_i] =      unk_counter * distance_weight + unk_num * absolute_weight;
        result.out_of_range[pose_i] = oor_num;

        const uint64 step = new_with_color ? 4 : 1;
        const uint64 r_offset = new_with_color ? 0 : 0;
        const uint64 g_offset = new_with_color ? 1 : 0;
        const uint64 b_offset = new_with_color ? 2 : 0;
        const uint64 a_offset = new_with_color ? 3 : 0;

        float max_alpha = 0.0;
        for (uint64 i = 0; i < image_size; i++)
          max_alpha = std::max(max_alpha,std::abs(imgvector[i * step + a_offset]));

        if (with_images)
        {
          result.images.push_back(sensor_msgs::Image());
          sensor_msgs::Image & image = result.images.back();

          image.height = new_size.y();
          image.width = new_size.x();
          image.encoding = "rgba8";
          image.is_bigendian = false;
          image.step = 4 * image.width;

          const uint64 size = image.height * image.width;
          image.data.resize(size * 4);

          if (new_with_color)
          {
            for (uint64 i = 0; i < size; i++)
            {
              image.data[i * 4 + 0] = imgvector[i * step + r_offset] * 255.9;
              image.data[i * 4 + 1] = imgvector[i * step + g_offset] * 255.9;
              image.data[i * 4 + 2] = imgvector[i * step + b_offset] * 255.9;
              image.data[i * 4 + 3] = imgvector[i * step + a_offset] / max_alpha * 127 + 128;
            }
          }
          else
          {
            for (uint64 i = 0; i < size; i++)
            {
              image.data[i * 4 + 0] = image.data[i * 4 + 1] = image.data[i * 4 + 2] = 0;
              image.data[i * 4 + 3] = imgvector[i * step + a_offset] / max_alpha * 127 + 128;
            }
          }
        }

        if (with_clouds)
        {
          const SurfelRenderer::GLFloatVector & depth_result = m_renderer->GetLastDepthResult();

          sensor_msgs::PointCloud2 pc;
          if (!new_with_color)
            pc = BuildCloudFromImage<pcl::PointXYZI,false>(new_size,new_center,new_focal,imgvector,step,depth_result,
                                                           r_offset,g_offset,b_offset,a_offset);
          else
            pc = BuildCloudFromImage<PointXYZIRGBA,true>(new_size,new_center,new_focal,imgvector,step,depth_result,
                                                         r_offset,g_offset,b_offset,a_offset);
          pc.header.frame_id = m_frame_id;
          result.clouds.push_back(pc);
        }

        counting_time += ros::Time::now() - counting_start_time;
      }

      const ros::Duration diff_time = ros::Time::now() - start_time;
      ROS_INFO_STREAM("surfel_next_best_view: poses evaluation completed in " << diff_time << "s :\n" <<
                      "  init:      " << check_opengl_time << " s\n" <<
                      "  gen cloud: " << cloud_gen_time << " s\n" <<
                      "  renderer:  " << generate_renderer_time << " s\n" <<
                      "  rendering: " << rendering_time << " s\n" <<
                      "  download:  " << download_time << " s\n" <<
                      "  counting:  " << counting_time << " s\n" <<
                      "  unknown:   " << (diff_time - (generate_renderer_time + check_opengl_time + cloud_gen_time +
                                                       rendering_time + download_time + counting_time)) << " s"
                      );


      {
        boost::mutex::scoped_lock lock(m_mutex);
        m_current_result = result_ptr;
        m_is_processing = false;
        m_current_goal.reset();
        m_cond.notify_all();
      }
    }
  }

  private:
  ros::NodeHandle & m_nh;
  bool m_opengl_context_initialized;
  std::string m_opengl_context_screen;

  boost::shared_ptr<EvaluatePosesActionServer> m_poses_as;
  boost::shared_ptr<SetPointCloudActionServer> m_cloud_as;
  ros::Subscriber m_point_cloud_sub;

  boost::mutex m_mutex;
  boost::condition_variable m_cond;
  bool m_is_processing;
  bool m_is_new_cloud;

  surfel_next_best_view_msgs::EvaluatePosesGoalConstPtr m_current_goal;
  surfel_next_best_view_msgs::EvaluatePosesResultConstPtr m_current_result;

  typedef pluginlib::ClassLoader<RenderPlugin> RenderPluginClassLoader;
  boost::shared_ptr<RenderPlugin> m_robot_urdf_filter;
  RenderPlugin * m_robot_urdf_filter_ptr;
  boost::shared_ptr<RenderPluginClassLoader> m_render_plugin_loader;

  SurfelRenderer::Ptr m_renderer;

  PointSurfelCloud::ConstPtr m_cloud;
  GPUSurfelCloud::ConstPtr m_cloud_gpu;

  std::string m_frame_id;
  bool m_enable_robot_filter;
  std::string m_enable_robot_filter_plugin;
};

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"surfel_next_best_view");
  ros::NodeHandle nh("~");

  SurfelNextBestView snbv(nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  snbv.Run();
  return 0;
}
