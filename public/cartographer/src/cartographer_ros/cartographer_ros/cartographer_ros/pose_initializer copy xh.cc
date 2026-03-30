/*
 * Copyright 2019 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string>
#include <vector>
#include <memory>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/GetTrajectoryStates.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gflags/gflags.h"
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h" 

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");

DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

DEFINE_string(load_state_filename, "",
              "Filename of a pbstream to draw a map from.");

namespace {
std::unique_ptr<cartographer::mapping::MapBuilder> map_builder_;
}  // namespace

// subscribe callback function from Rviz
void move_base_simple_callback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  ::ros::NodeHandle nh;
  
  // 创建TF buffer和listener
  static std::unique_ptr<tf2_ros::Buffer> tf_buffer = nullptr;
  static std::unique_ptr<tf2_ros::TransformListener> tf_listener = nullptr;
  
  if (!tf_buffer) {
    tf_buffer = std::make_unique<tf2_ros::Buffer>();
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
    ROS_INFO("TF buffer and listener initialized");
  }

  // ---------- 获取当前 active 轨迹 ----------
  ::ros::ServiceClient client_get_traj_states =
      nh.serviceClient<cartographer_ros_msgs::GetTrajectoryStates>(
          cartographer_ros::kGetTrajectoryStatesServiceName);

  cartographer_ros_msgs::GetTrajectoryStates srv_get_traj_states;
  if (!client_get_traj_states.call(srv_get_traj_states)) {
    ROS_ERROR("Failed to call GetTrajectoryStates.");
    return;
  }
  if (srv_get_traj_states.response.status.code !=
      cartographer_ros_msgs::StatusCode::OK) {
    ROS_ERROR("Error get trajectory states - %s",
              srv_get_traj_states.response.status.message.c_str());
    return;
  }

  int current_traj_id = -1;
  for (size_t i = 0; i < srv_get_traj_states.response.trajectory_states.trajectory_state.size(); ++i) {
    if (srv_get_traj_states.response.trajectory_states.trajectory_state[i] ==
        cartographer_ros_msgs::TrajectoryStates::ACTIVE) {
      current_traj_id = srv_get_traj_states.response.trajectory_states.trajectory_id[i];
      break;
    }
  }

  if (current_traj_id == -1) {
    ROS_ERROR("No active trajectory!");
    return;
  }

  ROS_INFO("Current active trajectory ID: %d", current_traj_id);

  // ---------- 结束当前轨迹 ----------
  ::ros::ServiceClient client_finish_traj =
      nh.serviceClient<cartographer_ros_msgs::FinishTrajectory>(
          cartographer_ros::kFinishTrajectoryServiceName);

  cartographer_ros_msgs::FinishTrajectory srv_finish_traj;
  srv_finish_traj.request.trajectory_id = current_traj_id;

  if (!client_finish_traj.call(srv_finish_traj)) {
    ROS_ERROR("Failed to call FinishTrajectory.");
    return;
  }
  if (srv_finish_traj.response.status.code !=
      cartographer_ros_msgs::StatusCode::OK) {
    ROS_ERROR("Error finishing trajectory - %s",
              srv_finish_traj.response.status.message.c_str());
    return;
  }

  ROS_INFO("Trajectory %d finished successfully", current_traj_id);

  // 等待 Collator 清空
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  ros::getGlobalCallbackQueue()->clear();

  // ---------- 处理RViz的initialpose ----------
  try {
    // 1. 获取base_link到imu_link的静态变换
    geometry_msgs::TransformStamped base_to_imu_tf;
    base_to_imu_tf = tf_buffer->lookupTransform("base_link", "imu_link", 
                                                 ros::Time(0), ros::Duration(2.0));
    
    // ROS_INFO("Got base_link->imu_link transform: [%.3f, %.3f, %.3f]",
    //          base_to_imu_tf.transform.translation.x,
    //          base_to_imu_tf.transform.translation.y,
    //          base_to_imu_tf.transform.translation.z);
    
    // 2. 将RViz给的initialpose（map->base_link）转换为map->imu_link
    tf2::Transform map_to_base_tf;
    tf2::fromMsg(msg->pose.pose, map_to_base_tf);
    
    tf2::Transform base_to_imu_tf2;
    tf2::fromMsg(base_to_imu_tf.transform, base_to_imu_tf2);
    
    // map->imu_link = map->base_link * base_link->imu_link
    tf2::Transform map_to_imu_tf = map_to_base_tf * base_to_imu_tf2;
    
    double yaw = tf2::getYaw(msg->pose.pose.orientation);
    // ROS_INFO("RViz pose (map->base_link): [%.3f, %.3f, %.3f] yaw: %.3f",
    //          msg->pose.pose.position.x,
    //          msg->pose.pose.position.y,
    //          msg->pose.pose.position.z,
    //          yaw);  
    
    // ROS_INFO("Computed map->imu_link: [%.3f, %.3f, %.3f]",
    //          map_to_imu_tf.getOrigin().x(),
    //          map_to_imu_tf.getOrigin().y(),
    //          map_to_imu_tf.getOrigin().z());
    
    // 3. 获取参考位姿（第一条轨迹的初始位姿，这是map->imu_link）
    const auto node_poses = map_builder_->pose_graph()->GetTrajectoryNodePoses();
    if (node_poses.empty()) {
      ROS_ERROR("No trajectory node poses found.");
      return;
    }
    
    const auto traj_ref_pose = node_poses.BeginOfTrajectory(0)->data.global_pose;
    tf2::Transform traj_ref_tf;
    traj_ref_tf.setOrigin(tf2::Vector3(
        traj_ref_pose.translation().x(),
        traj_ref_pose.translation().y(),
        traj_ref_pose.translation().z()));
    traj_ref_tf.setRotation(tf2::Quaternion(
        traj_ref_pose.rotation().x(),
        traj_ref_pose.rotation().y(),
        traj_ref_pose.rotation().z(),
        traj_ref_pose.rotation().w()));
    
    // ROS_INFO("Reference pose (map->imu_link from pbstream): [%.3f, %.3f, %.3f]",
    //          traj_ref_pose.translation().x(),
    //          traj_ref_pose.translation().y(),
    //          traj_ref_pose.translation().z());
    
    // 4. 计算相对变换：relative = (参考位姿的逆) * (新的初始位姿)
    tf2::Transform relative_initpose_tf = traj_ref_tf.inverse() * map_to_imu_tf;
    
    // ROS_INFO("Relative pose for StartTrajectory: [%.3f, %.3f, %.3f]",
    //          relative_initpose_tf.getOrigin().x(),
    //          relative_initpose_tf.getOrigin().y(),
    //          relative_initpose_tf.getOrigin().z());
    
    // ---------- 启动新轨迹 ----------
    ::ros::ServiceClient client_start_traj =
        nh.serviceClient<cartographer_ros_msgs::StartTrajectory>(
            cartographer_ros::kStartTrajectoryServiceName);

    cartographer_ros_msgs::StartTrajectory srv_start_traj;
    srv_start_traj.request.configuration_directory = FLAGS_configuration_directory;
    srv_start_traj.request.configuration_basename = FLAGS_configuration_basename;

    srv_start_traj.request.relative_to_trajectory_id = 0;
    srv_start_traj.request.use_initial_pose = true;
    tf2::toMsg(relative_initpose_tf, srv_start_traj.request.initial_pose);

    // 检测 /clock 模式
    bool use_sim_time = false;
    nh.getParam("/use_sim_time", use_sim_time);
    if (use_sim_time) {
      ROS_INFO("Detected simulated time (/clock). Ensure rosbag --clock started.");
    }

    // 重试机制
    int retry_count = 0;
    bool success = false;
    while (retry_count < 3) {
      if (client_start_traj.call(srv_start_traj) &&
          srv_start_traj.response.status.code ==
              cartographer_ros_msgs::StatusCode::OK) {
        ROS_INFO("New trajectory started successfully with ID: %d!", 
                 srv_start_traj.response.trajectory_id);
        success = true;
        break;
      } else {
        ROS_WARN("StartTrajectory failed, retrying (%d/3)...", retry_count + 1);
        if (srv_start_traj.response.status.code != 
            cartographer_ros_msgs::StatusCode::OK) {
          ROS_WARN("Error: %s", srv_start_traj.response.status.message.c_str());
        }
        ros::Duration(0.5).sleep();
        retry_count++;
      }
    }

    if (!success) {
      ROS_ERROR("Failed to start new trajectory after 3 attempts.");
    }
    
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("TF transform error: %s", ex.what());
    ROS_ERROR("Make sure TF tree contains base_link->imu_link transform");
    return;
  } catch (const std::exception& e) {
    ROS_ERROR("Exception: %s", e.what());
    return;
  }
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  
  google::SetUsageMessage(
      "\n\n"
      "Convenience tool around the start_trajectory service. This takes a Lua "
      "file that is accepted by the node as well and starts a new trajectory "
      "using its settings.\n");
  
  google::ParseCommandLineFlags(&argc, &argv, true);
  
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";
  
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  
  // load pbstream
  ::cartographer::io::ProtoStreamReader reader(FLAGS_load_state_filename);
  cartographer_ros::NodeOptions node_options;
  std::tie(node_options, std::ignore) = cartographer_ros::LoadOptions(
      FLAGS_configuration_directory, FLAGS_configuration_basename);
  map_builder_ = absl::make_unique<cartographer::mapping::MapBuilder>(
      node_options.map_builder_options);
  map_builder_->LoadState(&reader, true);
  
  ROS_INFO("Loaded pbstream from: %s", FLAGS_load_state_filename.c_str());
  
  ::ros::init(argc, argv, "pose_initializer");
  ::ros::start();
  
  ::ros::NodeHandle nh;
  ::ros::Subscriber sub_move_base_simple =
      nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
          "/initialpose", 1, &move_base_simple_callback);
  ::ros::spin();
  
  return 0;
}