# Cloud Info
Header header 
# 第 i 条激光扫描线（Ring）上的第一个点在 extractedCloud 点云中的索引值；快速定位每条扫描线的起点和终点
int32[] startRingIndex       
int32[] endRingIndex  

int32[]  pointColInd # point column index in range image 每个点投影到2D距离图像后，所在的列索引。用于快速判断两个点在图像上是否相邻
float32[] pointRange # point range  每个点相对于雷达自身的径向距离（或深度值）。用于快速计算点的曲率
 
int64 imuAvailable
int64 odomAvailable

# Attitude for LOAM initialization
float32 imuRollInit
float32 imuPitchInit
float32 imuYawInit

# Initial guess from imu pre-integration
# 通过IMU预积分技术计算出的、从上一帧激光雷达到当前帧激光雷达的相对位姿变换
float32 initialGuessX
float32 initialGuessY
float32 initialGuessZ
float32 initialGuessRoll
float32 initialGuessPitch
float32 initialGuessYaw

# Point cloud messages
sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed 原始点云
sensor_msgs/PointCloud2 cloud_corner    # extracted corner feature  角点特征
sensor_msgs/PointCloud2 cloud_surface   # extracted surface feature 面点特征

# 3rd party messages  建图调试
sensor_msgs/PointCloud2 key_frame_cloud
sensor_msgs/PointCloud2 key_frame_color
sensor_msgs/PointCloud2 key_frame_poses
sensor_msgs/PointCloud2 key_frame_map
