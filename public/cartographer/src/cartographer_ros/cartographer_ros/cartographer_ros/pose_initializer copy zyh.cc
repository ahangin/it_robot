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
 */ // 文件头部版权与许可声明

#include <string> // 引入 std::string
#include <vector> // 引入 std::vector

#include "cartographer/common/configuration_file_resolver.h" // Cartographer 配置文件解析
#include "cartographer/io/proto_stream.h" // Cartographer pbstream 读写
#include "cartographer/mapping/map_builder.h" // MapBuilder 类（管理地图/轨迹）
#include "cartographer_ros/node_constants.h" // Cartographer ROS 常量（如 service 名称）
#include "cartographer_ros/node_options.h" // 加载 ROS 节点相关选项
#include "cartographer_ros/ros_log_sink.h" // 将 Cartographer 日志输出到 ROS
#include "cartographer_ros_msgs/FinishTrajectory.h" // FinishTrajectory 服务消息头
#include "cartographer_ros_msgs/GetTrajectoryStates.h" // GetTrajectoryStates 服务消息头
#include "cartographer_ros_msgs/StartTrajectory.h" // StartTrajectory 服务消息头
#include "cartographer_ros_msgs/StatusCode.h" // 状态码定义消息头
#include "geometry_msgs/PoseWithCovarianceStamped.h" // RViz initialpose 使用的消息类型
#include "gflags/gflags.h" // Google flags（命令行参数解析）
#include "ros/ros.h" // ROS 基本头文件
#include <ros/callback_queue.h>
#include "std_msgs/String.h" // std_msgs/String 消息头（未直接使用但常见）
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" // tf2 与 geometry_msgs 的互转工具

DEFINE_string(configuration_directory, "", // 定义命令行参数：配置文件目录
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there."); // 参数说明

DEFINE_string(configuration_basename, "", // 定义命令行参数：配置文件名（不含路径）
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file."); // 参数说明

DEFINE_string(load_state_filename, "", // 定义命令行参数：要加载的 pbstream 文件名（可选）
              "Filename of a pbstream to draw a map from."); // 参数说明

namespace { // 匿名命名空间，内部文件作用域
std::unique_ptr<cartographer::mapping::MapBuilder> map_builder_; // 全局指针：MapBuilder 的实例（在 main 中创建并加载 pbstream）
}  // namespace

// subscribe callback function from Rviz
void move_base_simple_callback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  ::ros::NodeHandle nh;

  // ---------- 获取当前 active 轨迹 ----------
  ::ros::ServiceClient client_get_traj_states =
      nh.serviceClient<cartographer_ros_msgs::GetTrajectoryStates>(
          cartographer_ros::kGetTrajectoryStatesServiceName);

  cartographer_ros_msgs::GetTrajectoryStates srv_get_traj_states;
  if (!client_get_traj_states.call(srv_get_traj_states)) {
    LOG(ERROR) << "Failed to call GetTrajectoryStates.";
    return;
  }
  if (srv_get_traj_states.response.status.code !=
      cartographer_ros_msgs::StatusCode::OK) {
    LOG(ERROR) << "Error get trajectory states - "
               << srv_get_traj_states.response.status.message;
    return;
  }

  int current_traj_id = -1;
  for (size_t i = 0; i < srv_get_traj_states.response.trajectory_states.trajectory_state.size(); ++i) {
    if (srv_get_traj_states.response.trajectory_states.trajectory_state[i] ==
        cartographer_ros_msgs::TrajectoryStates::ACTIVE) {
      current_traj_id = srv_get_traj_states.response.trajectory_states.trajectory_id[i];
    }
  }

  if (current_traj_id == -1) {
    LOG(ERROR) << "No active trajectory!";
    return;
  }

  // ---------- 结束当前轨迹 ----------
  ::ros::ServiceClient client_finish_traj =
      nh.serviceClient<cartographer_ros_msgs::FinishTrajectory>(
          cartographer_ros::kFinishTrajectoryServiceName);

  cartographer_ros_msgs::FinishTrajectory srv_finish_traj;
  srv_finish_traj.request.trajectory_id = current_traj_id;  // 修改：不使用 ++

  if (!client_finish_traj.call(srv_finish_traj)) {
    LOG(ERROR) << "Failed to call FinishTrajectory.";
    return;
  }
  if (srv_finish_traj.response.status.code !=
      cartographer_ros_msgs::StatusCode::OK) {
    LOG(ERROR) << "Error finishing trajectory - "
               << srv_finish_traj.response.status.message;
    return;
  }

  // 修改 1：等待 Collator 清空
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  ros::getGlobalCallbackQueue()->clear(); // 清除旧消息，防止时间乱序

  // ---------- 计算相对位姿 ----------
  const auto node_poses = map_builder_->pose_graph()->GetTrajectoryNodePoses();
  if (node_poses.empty()) {
    LOG(ERROR) << "No trajectory node poses found.";
    return;
  }

  const auto traj_ref_pose = node_poses.BeginOfTrajectory(0)->data.global_pose;
  tf2::Transform traj_ref_tf;
  traj_ref_tf.setOrigin(tf2::Vector3(traj_ref_pose.translation().x(),
                                     traj_ref_pose.translation().y(),
                                     traj_ref_pose.translation().z()));
  traj_ref_tf.setRotation(tf2::Quaternion(
      traj_ref_pose.rotation().x(), traj_ref_pose.rotation().y(),
      traj_ref_pose.rotation().z(), traj_ref_pose.rotation().w()));

  tf2::Transform map_tf;
  tf2::fromMsg(msg->pose.pose, map_tf);
  tf2::Transform relative_initpose_tf = traj_ref_tf.inverse() * map_tf;

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

  // 修改 2：检测 /clock 模式并打印提示
  bool use_sim_time = false;
  nh.getParam("/use_sim_time", use_sim_time);
  if (use_sim_time)
    LOG(INFO) << "[pose_initializer] Detected simulated time (/clock). Ensure rosbag --clock started.";

  // 修改 3：增加重试机制（防止服务没准备好）
  int retry_count = 0;
  while (retry_count < 3) {
    if (client_start_traj.call(srv_start_traj) &&
        srv_start_traj.response.status.code ==
            cartographer_ros_msgs::StatusCode::OK) {
      LOG(INFO) << "New trajectory started successfully!";
      break;
    } else {
      LOG(WARNING) << "StartTrajectory failed, retrying (" << retry_count + 1
                   << "/3)...";
      ros::Duration(0.5).sleep();
      retry_count++;
    }
  }

  if (retry_count == 3) {
    LOG(ERROR) << "Failed to start new trajectory after 3 attempts.";
  }
}


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]); // 初始化 Google logging（glog）

  google::SetUsageMessage(
      "\n\n"
      "Convenience tool around the start_trajectory service. This takes a Lua "
      "file that is accepted by the node as well and starts a new trajectory "
      "using its settings.\n"); // 设置程序使用说明（gflags 用）

  google::ParseCommandLineFlags(&argc, &argv, true); // 解析命令行参数（gflags）

  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing."; // 确保配置文件名参数非空，否则终止并打印错误

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing."; // 确保配置目录参数非空，否则终止并打印错误

  // load pbstream
  ::cartographer::io::ProtoStreamReader reader(FLAGS_load_state_filename); // 使用 ProtoStreamReader 打开 pbstream 文件（按 FLAGS_load_state_filename）
  cartographer_ros::NodeOptions node_options; // 声明 NodeOptions 对象（将被 LoadOptions 填充）
  std::tie(node_options, std::ignore) = cartographer_ros::LoadOptions(
      FLAGS_configuration_directory, FLAGS_configuration_basename); // 从 Lua 文件加载 Cartographer 配置选项
  map_builder_ = absl::make_unique<cartographer::mapping::MapBuilder>(
      node_options.map_builder_options); // 使用读取的 map_builder_options 构造 MapBuilder 实例
  map_builder_->LoadState(&reader, true); // 加载 pbstream 到 map_builder（恢复地图/轨迹状态）

  ::ros::init(argc, argv, "pose_initializer"); // 初始化 ROS 节点（名字 pose_initializer）
  ::ros::start(); // 启动 ROS（用于结合非 ROS main 环境）

  ::ros::NodeHandle nh; // 创建 NodeHandle（用于订阅 topic）
  ::ros::Subscriber sub_move_base_simple =
      nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
          "/initialpose", 1, &move_base_simple_callback); // 订阅 /initialpose（RViz 的 2D Pose Estimate），回调为 move_base_simple_callback
  ::ros::spin(); // 进入 ROS 循环，等待回调
}
