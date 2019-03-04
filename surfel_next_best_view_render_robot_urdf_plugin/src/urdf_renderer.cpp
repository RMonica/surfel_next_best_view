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

// GLEW
#include <GL/glew.h>

// ROS
#include <ros/node_handle.h>

// URDF
#include "urdf_renderer.h"

// Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <tf_conversions/tf_eigen.h>

// MOVEIT
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// MSG
#include <sensor_msgs/JointState.h>

namespace realtime_urdf_filter
{
  // costructor
  URDFRenderer::URDFRenderer (std::string description_param,
            std::string model_description)
    : model_description_(model_description)
    , robot_model_loader_(description_param)
    , robot_state_(robot_model_loader_.getModel())
  {
    robot_state_.setToDefaultValues();
    initURDFModel ();
    robot_position_ = Eigen::Affine3d::Identity();
  }

  // getter
  std::string URDFRenderer::getModelDescription (){return model_description_;}

  // loads URDF model from the parameter server and parses it. call loadURDFModel 
  void URDFRenderer::initURDFModel()
  {
    urdf::Model model;
    if (!model.initString(model_description_))
    {
      ROS_ERROR ("URDF failed Model parse");
      return;
    }

    ROS_INFO ("URDF parsed OK");
    loadURDFModel (model);
    ROS_INFO ("URDF loaded OK");
  }


  // load URDF model description from string and create search operations data structures
  void URDFRenderer::loadURDFModel
    (urdf::Model &model)
  {
    typedef std::vector<boost::shared_ptr<urdf::Link> > V_Link;
    V_Link links;
    model.getLinks(links);

    V_Link::iterator it = links.begin();
    V_Link::iterator end = links.end();

    for (; it != end; ++it)
      process_link (*it);
  }

  // processes a single URDF link, creates renderable for it */
  void URDFRenderer::process_link (boost::shared_ptr<urdf::Link> link)
  {
    if (link->visual.get() == NULL || link->visual->geometry.get() == NULL)
      return;

    boost::shared_ptr<Renderable> r;
    if (link->visual->geometry->type == urdf::Geometry::BOX)
    {
      boost::shared_ptr<urdf::Box> box = boost::dynamic_pointer_cast<urdf::Box> (link->visual->geometry);
      r.reset (new RenderableBox (box->dim.x, box->dim.y, box->dim.z));
    }
    else if (link->visual->geometry->type == urdf::Geometry::CYLINDER)
    {
      boost::shared_ptr<urdf::Cylinder> cylinder = boost::dynamic_pointer_cast<urdf::Cylinder> (link->visual->geometry);
      r.reset (new RenderableCylinder (cylinder->radius, cylinder->length));
    }
    else if (link->visual->geometry->type == urdf::Geometry::SPHERE)
    {
      boost::shared_ptr<urdf::Sphere> sphere = boost::dynamic_pointer_cast<urdf::Sphere> (link->visual->geometry);
      r.reset (new RenderableSphere (sphere->radius));
    }
    else if (link->visual->geometry->type == urdf::Geometry::MESH)
    {
      boost::shared_ptr<urdf::Mesh> mesh = boost::dynamic_pointer_cast<urdf::Mesh> (link->visual->geometry);
      std::string meshname (mesh->filename);
      RenderableMesh* rm = new RenderableMesh (meshname);
      rm->setScale (mesh->scale.x, mesh->scale.y, mesh->scale.z);
      r.reset (rm);
    }
    r->setLinkName (link->name);
    urdf::Vector3 origin = link->visual->origin.position;
    urdf::Rotation rotation = link->visual->origin.rotation;
    r->link_offset = tf::Transform(tf::Quaternion (rotation.x, rotation.y, rotation.z, rotation.w).normalize (),
				   tf::Vector3 (origin.x, origin.y, origin.z));
    if (link->visual && 
        (link->visual->material))
      r->color  = link->visual->material->color;
    renderables_.push_back (r); 
  }

  void URDFRenderer::update_link_transforms ()
  {
    std::vector<boost::shared_ptr<Renderable> >::const_iterator it = renderables_.begin ();
    for (; it != renderables_.end (); it++){
      //tf::transformEigenToTF (const Eigen::Affine3d &e, tf::Transform &t);
      tf::Transform transform;
      
      const Eigen::Affine3d robot_joint = robot_state_.getFrameTransform((*it)->name);
      const Eigen::Affine3d mat_joint = robot_position_ * robot_joint;
      tf::transformEigenToTF(mat_joint, transform);
      //check transform
      
      (*it)->link_to_fixed = transform; //Robot state
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief loops over all renderables and renders them to canvas */
  void URDFRenderer::render (){
    update_link_transforms ();

    std::vector<boost::shared_ptr<Renderable> >::const_iterator it = renderables_.begin ();
    for (; it != renderables_.end (); it++){
      (*it)->render ();
    }
  }

  void URDFRenderer::setJointRobot(sensor_msgs::JointState joint_state, const Eigen::Affine3d & robot_position){
    try
    {
      robot_state_.setVariablePositions(joint_state.name, joint_state.position);
      robot_position_ = robot_position;
    }
    catch (moveit::Exception e)
    {
      ROS_ERROR("surfel_next_best_view: setJointRobot: could not set joint states: %s",e.what());
    }
  }
}

