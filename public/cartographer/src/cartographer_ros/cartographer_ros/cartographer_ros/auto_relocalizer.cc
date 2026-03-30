#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>   // 支持点云触发
#include <sensor_msgs/LaserScan.h>     // 雷达触发
#include <ros/package.h>
// ------------------------------
// 参数配置结构体
// ------------------------------
struct CandidatePose {
  double x, y, z;
  double qx, qy, qz, qw;
};

// ------------------------------
// 主类定义
// ------------------------------
class PoseInitializer {
public:
  PoseInitializer(ros::NodeHandle& nh) : nh_(nh) {
    nh_.param("score_threshold", score_threshold_, 0.6);  // 匹配分数阈值
    nh_.param("timeout_sec", timeout_sec_, 5.0);          // 每个候选位姿超时时间
    nh_.param("pose_yaml_path",
              pose_yaml_path_,
              std::string("/home/cst/cartographer/carto_ws/src/cartographer_ros/cartographer_ros/params/candidate_poses.yaml"));

    // 添加用于触发的激光话题
    nh_.param<std::string>("lidar_topic", lidar_topic_, "/cloud_deskewed");  // 可改为 /scan

    pub_init_pose_ =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);
    sub_score_ = nh_.subscribe("/auto_relocalizer_score", 1, &PoseInitializer::ScoreCallback, this);

    // 订阅激光触发
    sub_lidar_ = nh_.subscribe(lidar_topic_, 1, &PoseInitializer::LidarCallback, this);

    got_score_ = false;
    got_first_lidar_ = false;
    last_score_ = 0.0;
    waiting_for_score_ = false;

    LoadCandidatePoses();
  }

  // ------------------------------
  // 候选位姿文件加载
  // ------------------------------
  void LoadCandidatePoses() {
    try {
      YAML::Node config = YAML::LoadFile(pose_yaml_path_);
      const auto& poses = config["candidate_poses"];
      for (size_t i = 0; i < poses.size(); ++i) {
        CandidatePose p;
        p.x = poses[i][0].as<double>();
        p.y = poses[i][1].as<double>();
        p.z = poses[i][2].as<double>();
        p.qx = poses[i][3].as<double>();
        p.qy = poses[i][4].as<double>();
        p.qz = poses[i][5].as<double>();
        p.qw = poses[i][6].as<double>();
        candidate_poses_.push_back(p);
      }
      ROS_INFO("[PoseInitializer] Loaded %zu candidate poses from %s",
               candidate_poses_.size(), pose_yaml_path_.c_str());
    } catch (const std::exception& e) {
      ROS_ERROR("[PoseInitializer] Failed to load candidate poses YAML: %s", e.what());
    }
  }

  // ------------------------------
  // Lidar 回调（触发计时）
  // ------------------------------
  void LidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!got_first_lidar_) {
      got_first_lidar_ = true;
      ROS_INFO("[PoseInitializer] First lidar message received, starting localization check...");
    }
  }

  // ------------------------------
  // 匹配分数回调
  // ------------------------------
  void ScoreCallback(const std_msgs::Float32::ConstPtr& msg) {
    if (!waiting_for_score_) {
      ROS_WARN("[PoseInitializer] Ignored score %.3f (not waiting).", msg->data);
      return;
    }

    last_score_ = msg->data;
    got_score_ = true;
    waiting_for_score_ = false;

    if (last_score_ >= score_threshold_) {
      ROS_INFO("[PoseInitializer] Received valid score: %.3f (threshold: %.3f). Localization confirmed.", last_score_, score_threshold_);
      ros::shutdown();  // 成功匹配，立即退出
    } else {
      ROS_WARN("[PoseInitializer] Score %.3f < %.3f, trying next candidate...", last_score_, score_threshold_);
    }
  }

  // ------------------------------
  // 发布 initialpose（并开启等待标志）
  // ------------------------------
  void PublishInitialPose(const CandidatePose& p) {
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();

    msg.pose.pose.position.x = p.x;
    msg.pose.pose.position.y = p.y;
    msg.pose.pose.position.z = p.z;
    msg.pose.pose.orientation.x = p.qx;
    msg.pose.pose.orientation.y = p.qy;
    msg.pose.pose.orientation.z = p.qz;
    msg.pose.pose.orientation.w = p.qw;

    pub_init_pose_.publish(msg);

    last_publish_time_ = ros::Time::now();
    waiting_for_score_ = true;  // 等待新的匹配分数
    ROS_INFO("[PoseInitializer] Published candidate pose: (%.2f, %.2f, %.2f) q=(%.2f, %.2f, %.2f, %.2f)",
             p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw);
  }

  // ------------------------------
  // 核心逻辑
  // ------------------------------
  void Run() {
    ROS_INFO("[PoseInitializer] Waiting for lidar data to start...");
    while (ros::ok() && !got_first_lidar_) {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }

    ROS_INFO("[PoseInitializer] First lidar message received, starting localization check...");

    ros::Time start_check = ros::Time::now();
    double initial_check_duration = 3.0;  // 检查前 3 秒是否已有高score
    got_score_ = false;
    last_score_ = 0.0;
    waiting_for_score_ = true;

    while (ros::ok() && (ros::Time::now() - start_check).toSec() < initial_check_duration) {
      ros::spinOnce();

      if (got_score_ && last_score_ >= score_threshold_) {
        ROS_INFO("[PoseInitializer] Already localized at startup (score=%.3f). Skip auto relocalization.",
                 last_score_);
        ros::shutdown();
        return;  // 直接退出
      }

      ros::Duration(0.1).sleep();
    }

    // 如果没有找到匹配的分数，进入候选位姿轮询
    ROS_INFO("[PoseInitializer] Not localized yet, start candidate loop...");

    waiting_for_score_ = false;

    for (size_t i = 0; i < candidate_poses_.size(); ++i) {
      const auto& pose = candidate_poses_[i];
      ROS_INFO("\n=============================\n[PoseInitializer] Trying candidate #%zu\n=============================", i + 1);

      got_score_ = false;
      last_score_ = 0.0;

      PublishInitialPose(pose);
      ros::Time start = ros::Time::now();

      while (ros::ok() && (ros::Time::now() - start).toSec() < timeout_sec_) {
        ros::spinOnce();

        if (got_score_) {
          if (last_score_ >= score_threshold_) {
            ROS_INFO("[PoseInitializer] Success! Score = %.3f >= %.3f. Localization confirmed.", last_score_, score_threshold_);
            ros::shutdown();
            return;
          } else {
            ROS_WARN("[PoseInitializer] Score %.3f < %.3f, trying next candidate...", last_score_, score_threshold_);
            break;
          }
        }

        ros::Duration(0.1).sleep();
      }

      if (!got_score_) {
        ROS_WARN("[PoseInitializer] Timeout (%.1fs) — no score received, next candidate.", timeout_sec_);
      }
    }

    ROS_ERROR("[PoseInitializer] No candidate pose succeeded! Please check map or thresholds.");
    ros::shutdown();
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_init_pose_;
  ros::Subscriber sub_score_;
  ros::Subscriber sub_lidar_;
  std::vector<CandidatePose> candidate_poses_;
  bool got_score_, got_first_lidar_;
  double last_score_, score_threshold_, timeout_sec_;
  std::string pose_yaml_path_;
  std::string lidar_topic_;
  bool waiting_for_score_;
  ros::Time last_publish_time_;
};

// ------------------------------
// 主函数
// ------------------------------
int main(int argc, char** argv) {
  ros::init(argc, argv, "auto_relocalizer");
  ros::NodeHandle nh("~");

  PoseInitializer initializer(nh);
  initializer.Run();

  return 0;
}
