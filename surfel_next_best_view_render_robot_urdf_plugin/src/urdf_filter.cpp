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

// URDF
#include "urdf_filter.h"

using namespace realtime_urdf_filter;

/* constructor. sets up ros and reads in parameters */
RealtimeURDFFilter::RealtimeURDFFilter (ros::NodeHandle &nh,const Eigen::Vector2i & image_size)
  : nh_ (nh), image_size_(image_size)
{
  ROS_INFO("surfel_next_best_view: realtime_urdf_filter loading.");

  nh_.param<bool>("draw_robot_depth", draw_robot_depth_, true);
  ROS_INFO ("draw_robot_depth: %s", (draw_robot_depth_ ? "ON" : "OFF"));

  /* depth distance threshold (how far from the model are points still deleted?) */
  nh_.param<double> ("depth_distance_threshold", depth_distance_threshold_, 0.0);
  ROS_INFO ("using depth distance threshold %f", depth_distance_threshold_);

  initGL();
}

/* destructor */
RealtimeURDFFilter::~RealtimeURDFFilter (){
  ROS_INFO("realtime_urdf_filter- RealtimeURDFFilter::~RealtimeURDFFilter () ");
}

/* loads URDF models */
void RealtimeURDFFilter::loadModels (){
  ROS_INFO("realtime_urdf_filter- RealtimeURDFFilter::loadModels");
  XmlRpc::XmlRpcValue xmlRpcValue;
  nh_.getParam ("models", xmlRpcValue);
  
  if (xmlRpcValue.getType () == XmlRpc::XmlRpcValue::TypeArray){
    for (int i = 0; i < xmlRpcValue.size(); ++i){
      XmlRpc::XmlRpcValue elem = xmlRpcValue[i];
      ROS_ASSERT (elem.getType()  == XmlRpc::XmlRpcValue::TypeStruct);

      std::string description_param = elem["model"];

      // read URDF model
      std::string content;

      if (!nh_.getParam(description_param, content)){
        ROS_ERROR ("Parameter [%s] does not exist",
            description_param.c_str());
        continue;
      }

      if (content.empty() || content==""){
        ROS_ERROR ("URDF is empty");
        continue;
      }

      /* finally, set the model description so we can later parse it */
      ROS_INFO ("Loading URDF model: %s", description_param.c_str ());
      renderers_.push_back(URDFRenderer::Ptr(new URDFRenderer (description_param, content)));
    }
  }
  else{
    ROS_ERROR ("models parameter must be an array!");
  }
}

void RealtimeURDFFilter::renderRobot(const sensor_msgs::JointState &joint_state, const Eigen::Affine3d & robot_position)
{
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glEnable(GL_NORMALIZE);
  glViewport(0,0,image_size_.x(),image_size_.y());
  
  // setup camera projection
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity();
   
  // setup camera position
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glPushMatrix();
  
  // render every renderable / urdf model
  for (std::vector<URDFRenderer::Ptr>::const_iterator r = renderers_.begin (); r != renderers_.end (); r++) {
    (*r)->setJointRobot(joint_state,robot_position);
    (*r)->render ();
  }
  
  glPopMatrix();
  glPopAttrib();
}

// set up OpenGL stuff
void RealtimeURDFFilter::initGL (){
  ROS_INFO("realtime_urdf_filter- RealtimeURDFFilter::initGL ()");

  // load URDF models 
  loadModels();

  // make sure we loaded something!
  if(renderers_.empty())
    throw std::runtime_error("Could not load any models for filtering!");
  else 
    ROS_INFO_STREAM("Loaded "<<renderers_.size()<<" models for filtering.");  
}

double RealtimeURDFFilter::getDepthDistanceThreshold()
{
  return depth_distance_threshold_;
}

