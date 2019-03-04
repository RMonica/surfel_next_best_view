/*
 * Written in 2018 by Francesco Periti and Riccardo Monica
 *   RIMLab, Department of Engineering and Architecture, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * To the extent possible under law, the author(s) have dedicated all copyright
 * and related and neighboring rights to this software to the public domain
 * worldwide. This software is distributed without any warranty.
 *
 * You should have received a copy of the CC0 Public Domain Dedication along
 * with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
 */

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <iostream>

// MOVEIT
#include <moveit/robot_state/robot_state.h> 
#include <moveit/robot_model_loader/robot_model_loader.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// custom
#include <surfel_next_best_view_msgs/EvaluatePosesAction.h>
#include <surfel_next_best_view_msgs/SetPointCloudAction.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>

//STD
#include <vector>

#define SERVER_WAIT_TIME 3.0
#define ANSWER_WAIT_TIME 10.0
#define CLOUD_SERVER "/surfel_next_best_view/set_point_cloud"
#define POSES_SERVER "/surfel_next_best_view/evaluate_poses"

typedef pcl::PointCloud<pcl::PointSurfel> PointSurfelCloud;
typedef unsigned int uint;

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"surfel_next_best_view_test");
  ros::NodeHandle nh("~");

  ros::Publisher test_image_publisher = nh.advertise<sensor_msgs::Image>("/surfel_next_best_view_test_image",1);
  ros::Publisher test_alpha_publisher = nh.advertise<sensor_msgs::Image>("/surfel_next_best_view_test_alpha",1);
  ros::Publisher test_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/surfel_next_best_view_test_cloud",1);

  std::string filename;
  nh.param<std::string>("cloud_filename",filename,"cloud.pcd");
  PointSurfelCloud cloud;
  ROS_INFO("Loading file: %s",filename.c_str());
  if (pcl::io::loadPCDFile(filename,cloud))
  {
    ROS_ERROR("Load failed.");
    return 67;
  }
  ROS_INFO("Loaded %d points.",int(cloud.size()));

  actionlib::SimpleActionClient<surfel_next_best_view_msgs::SetPointCloudAction>
    ac_cloud(CLOUD_SERVER,true);
  actionlib::SimpleActionClient<surfel_next_best_view_msgs::EvaluatePosesAction>
    ac_poses(POSES_SERVER, true);

  ROS_INFO("Waiting %f seconds for action servers to start.",float(SERVER_WAIT_TIME));
  if (!ac_cloud.waitForServer(ros::Duration(SERVER_WAIT_TIME)))
  {
    ROS_ERROR("Could not connect to server %s!",CLOUD_SERVER);
    return 1;
  }
  if (!ac_poses.waitForServer(ros::Duration(SERVER_WAIT_TIME)))
  {
    ROS_ERROR("Could not connect to server %s!",POSES_SERVER);
    return 1;
  }

  ROS_INFO("Setting cloud...");
  surfel_next_best_view_msgs::SetPointCloudGoal cloud_goal;
  pcl::toROSMsg(cloud,cloud_goal.cloud);
  ac_cloud.sendGoal(cloud_goal);

  ac_cloud.waitForResult(ros::Duration(ANSWER_WAIT_TIME));
  actionlib::SimpleClientGoalState state = ac_cloud.getState();
  if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_ERROR("Action did not succeed, state is %s.",state.toString().c_str());
    return 3;
  }
  ROS_INFO("Cloud set.");
  
  std::vector<Eigen::Affine3f,Eigen::aligned_allocator<Eigen::Affine3f> > test_poses;
  
  Eigen::Affine3f test_pose = Eigen::Affine3f::Identity();

  Eigen::Quaternionf fromWorld_toCamera = Eigen::Quaternionf(0.044, 0.328, 0.729, -0.600);
  Eigen::Matrix3f rot = fromWorld_toCamera.normalized().toRotationMatrix();		
  test_pose.translation() = Eigen::Vector3f(0.499, 0.900, 0.843);
  test_pose.linear() = rot;

  test_poses.push_back(test_pose);

  surfel_next_best_view_msgs::EvaluatePosesGoal poses_goal;

  for (uint i = 0; i < test_poses.size(); i++)
  {
    geometry_msgs::Pose pose_msg;
    ROS_INFO_STREAM("Prepared test pose at\n" << test_poses[i].matrix() << "\n");
    tf::poseEigenToMsg(test_poses[i].cast<double>(),pose_msg);
    poses_goal.poses.push_back(pose_msg);
  }
  
  // current robot joints
  std::vector<std::string> name = {"joint_1",
                                   "joint_2",
                                   "joint_3",
                                   "joint_4",
                                   "joint_5",
                                   "joint_6"};

  std::vector<double> position = { -0.9324072521518851,
                                    0.43188173805338287,
                                   -1.1011282277463836,
                                    2.0274268541478997,
                                   -1.7433396043439315,
                                   -2.7820025231951524 };
  
  sensor_msgs::JointState joint_state;
  joint_state.name = name;
  joint_state.position = position;
  poses_goal.joint_states.push_back(joint_state);
  
  poses_goal.unknown_score = 1.0;
  poses_goal.occupied_score = 0.0;
  poses_goal.width = 640;
  poses_goal.height = 480;
  poses_goal.focal_x = 525;
  poses_goal.focal_y = 525;
  poses_goal.center_x = 320;
  poses_goal.center_y = 240;
  poses_goal.range_min = 0.1;
  poses_goal.range_max = 12.0;
  poses_goal.with_images = true;
  poses_goal.with_color = true;
  poses_goal.with_clouds = true;
  
  ROS_INFO("Sending goal (1)...");
  ac_poses.sendGoal(poses_goal);

  ROS_INFO("Waiting for result...");
  ac_poses.waitForResult(ros::Duration(ANSWER_WAIT_TIME));

  actionlib::SimpleClientGoalState state_poses = ac_poses.getState();
  if (state_poses != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_ERROR("Action did not succeed, state is %s.",state_poses.toString().c_str());
    return 6;
  }

  ROS_INFO("Result:");
  surfel_next_best_view_msgs::EvaluatePosesResult result = *ac_poses.getResult();

  for (uint i = 0; i < result.scores.size(); i++)
  {
    ROS_INFO("  pose %u -> score              %f",uint(i),float(result.scores[i]));
    ROS_INFO("          -> occupied           %f",float(result.occupied[i]));
    ROS_INFO("          -> unknown            %f",float(result.unknown[i]));
    ROS_INFO("          -> out_of_range       %f",float(result.out_of_range[i]));
  }

  for (uint i = 0; i < result.images.size(); i++)
  {
    ROS_INFO("  publishing image %d",int(i));
    test_image_publisher.publish(result.images[i]);

    pcl::PointCloud<pcl::PointXYZ> point_cloud1;
    pcl::fromROSMsg(result.clouds[i], point_cloud1);
    sensor_msgs::Image img = result.images[i];

    for (uint i = 0; i < img.data.size() / 4; i++)
    {
      const uint alpha = img.data[i * 4 + 3];
      img.data[i * 4 + 0] = alpha;
      img.data[i * 4 + 1] = alpha;
      img.data[i * 4 + 2] = alpha;
      img.data[i * 4 + 3] = 255;
    }

    test_alpha_publisher.publish(img);
    test_cloud_publisher.publish(result.clouds[i]);
    ros::Duration(0.3).sleep();
  }

  ros::spin();
  return 0;
}
