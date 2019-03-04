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

#ifndef REALTIME_MY_URDF_FILTER_URDF_RENDERER_H_
#define REALTIME_MY_URDF_FILTER_URDF_RENDERER_H_

// URDF 
#include <urdf/model.h>
#include "renderable.h"

// MOVEIT
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <boost/shared_ptr.hpp>

// forward declares
namespace ros {class NodeHandle;}

namespace realtime_urdf_filter
{
  
  class URDFRenderer
  { 
  public:
    typedef boost::shared_ptr<URDFRenderer> Ptr;

    // costructor
    URDFRenderer (std::string description_param,std::string model_description);

    // for render
    void update_link_transforms ();
    void render ();

    // getter
    std::string getModelDescription();

    void setJointRobot(sensor_msgs::JointState joint_state, const Eigen::Affine3d & robot_position);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  protected:
    // init URDFModel
    void initURDFModel ();
    // load URDFModel
    void loadURDFModel (urdf::Model &descr);
    
    void process_link (boost::shared_ptr<urdf::Link> link);   

    // urdf model stuff
    std::string model_description_;
   
    // rendering stuff 
    std::vector<boost::shared_ptr<Renderable> > renderables_;

    // Robot state
    robot_model_loader::RobotModelLoader robot_model_loader_;
    robot_state::RobotState robot_state_;
    Eigen::Affine3d robot_position_;
  };

} // end namespace

#endif //REALTIME_MY_URDF_FILTER_URDF_RENDERER_H_
