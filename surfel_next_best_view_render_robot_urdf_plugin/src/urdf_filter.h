/*
 * Copyright (c) 2011, Nico Blodow <blodow@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef REALTIME_URDF_FILTER_URDF_FILTER_H_
#define REALTIME_URDF_FILTER_URDF_FILTER_H_

// GLEW
#include <GL/glew.h>

// ROS
#include <ros/node_handle.h>

// MSG
#include <sensor_msgs/JointState.h>

// URDF
#include "urdf_renderer.h"

#include <Eigen/Dense>


namespace realtime_urdf_filter
{

class RealtimeURDFFilter
{
  public:
    // constructor. sets up ros and reads in parameters
    RealtimeURDFFilter (ros::NodeHandle &nh,const Eigen::Vector2i & image_size);

    // destructor
    ~RealtimeURDFFilter ();

    // loads URDF models
    void loadModels ();

    // set up OpenGL stuff
    void initGL ();

    // render robot
    void renderRobot(const sensor_msgs::JointState &joint_state, const Eigen::Affine3d & robot_position);

    // getter
    double getDepthDistanceThreshold();
    int getWidth();
    int getHeight();
    bool getDrawRobotDepth() const {return draw_robot_depth_; }


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
  private:
    // ROS objects
    ros::NodeHandle & nh_;
    Eigen::Vector2i image_size_;

    // vector of renderables
    std::vector<URDFRenderer::Ptr> renderers_;

    // parameters
    bool draw_robot_depth_;
    double depth_distance_threshold_;
};
 
} // end namespace

#endif // REALTIME_URDF_FILTER_URDF_FILTER_H_ 
