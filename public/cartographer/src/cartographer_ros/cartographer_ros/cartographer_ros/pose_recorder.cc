#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ctime>
#include <string>
#include <mutex>

class PoseRecorder {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    geometry_msgs::PoseStamped latest_pose_;
    std::string output_file_;
    double update_interval_;
    bool has_new_pose_;
    std::mutex pose_mutex_;
    
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_pose_ = *msg;
        has_new_pose_ = true;
        ROS_DEBUG("Received new pose");
    }

    void savePoseToYaml() {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        
        if (!has_new_pose_) {
            return;
        }
        
        try {
            YAML::Node pose_node;
            
            // 添加初始位姿参数（直接使用 set_inital_pose_* 格式）
            pose_node["set_initial_pose_x"]  = latest_pose_.pose.position.x;
            pose_node["set_initial_pose_y"]  = latest_pose_.pose.position.y;
            pose_node["set_initial_pose_z"]  = latest_pose_.pose.position.z;
            pose_node["set_initial_pose_ox"] = latest_pose_.pose.orientation.x;
            pose_node["set_initial_pose_oy"] = latest_pose_.pose.orientation.y;
            pose_node["set_initial_pose_oz"] = latest_pose_.pose.orientation.z;
            pose_node["set_initial_pose_ow"] = latest_pose_.pose.orientation.w;

            // 添加记录时间
            // std::time_t now = std::time(nullptr);
            // char time_str[100];
            // std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
            // pose_node["recorded_at"] = time_str;
            
            // 保存到文件
            std::ofstream fout(output_file_);
            if (fout.is_open()) {
                fout << pose_node;
                fout.close();
                ROS_DEBUG("Pose saved to %s", output_file_.c_str());
            } else {
                ROS_ERROR("Failed to open file: %s", output_file_.c_str());
            }
            
            has_new_pose_ = false;
        } catch (const std::exception& e) {
            ROS_ERROR("Error saving pose to YAML: %s", e.what());
        }
    }
    
public:
    PoseRecorder() : has_new_pose_(false) {
        // 获取参数
        ros::NodeHandle private_nh("~");
        private_nh.param("update_interval", update_interval_, 1.0);
        private_nh.param("output_file", output_file_, 
                        std::string("/home/cst/carto_ws/src/cartographer_ros/cartographer_ros/params/pose.yaml"));
        
        // 确保输出目录存在
        std::string output_dir = output_file_.substr(0, output_file_.find_last_of("/"));
        std::string mkdir_cmd = "mkdir -p " + output_dir;
        if (system(mkdir_cmd.c_str()) != 0) {
            ROS_WARN("Failed to create directory: %s", output_dir.c_str());
        }
        
        // 订阅话题
        pose_sub_ = nh_.subscribe("/tracked_pose", 10, &PoseRecorder::poseCallback, this);
        
        ROS_INFO("Pose recorder started. Saving to: %s every %.1f seconds", 
                 output_file_.c_str(), update_interval_);
    }
    
    void run() {
        ros::Rate rate(1.0 / update_interval_);
        while (ros::ok()) {
            savePoseToYaml();
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_recorder");
    PoseRecorder recorder;
    recorder.run();
    return 0;
}
