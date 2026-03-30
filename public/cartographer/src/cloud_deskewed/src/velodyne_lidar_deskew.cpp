#include "utility.h"
#include "cloud_deskewed/cloud_info.h"

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

// rslidar的点云格式
struct RsPointXYZIRT {
    PCL_ADD_POINT4D;
    // uint8_t intensity;
    float  intensity;
    uint16_t ring = 0;
    double timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                          (uint16_t, ring, ring)(double, timestamp, timestamp))


struct OusterPointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(uint8_t, ring, ring)(uint16_t, noise, noise)(uint32_t, range, range))

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class ImageProjection : public ParamServer
{
private:
    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher pubLaserCloud;

    ros::Publisher pubExtractedCloud;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<RsPointXYZIRT>::Ptr rsLaserCloudIn;
    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    cloud_deskewed::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

    vector<int> columnIdnCountVec;

public:
    ImageProjection() : deskewFlag(0)
    {
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        // subOdom = nh.subscribe<nav_msgs::Odometry>(odomTopic + "_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2>("cloud_deskewed", 1);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        rsLaserCloudIn.reset(new pcl::PointCloud<RsPointXYZIRT>());

    

        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN * Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        cloudInfo.pointColInd.assign(N_SCAN * Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN * Horizon_SCAN, 0);

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }

        columnIdnCountVec.assign(N_SCAN, 0);
    }

    ~ImageProjection() {}

    void imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg)
    {
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x <<
        //       ", y: " << thisImu.linear_acceleration.y <<
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x <<
        //       ", y: " << thisImu.angular_velocity.y <<
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        
        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();

        cloudExtraction();

        publishClouds();

        resetParameters();
    }
    // 数据检查，适配不同雷达
    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {

        // 1. 缓存点云消息到队列
        // 将新到来的点云消息存入 cloudQueue 队列的尾部
        cloudQueue.push_back(*laserCloudMsg);

        // 2. 队列长度检查
        // 如果队列中的点云数量小于等于2，说明数据还不够，暂时不进行处理。
        // 这是因为后续的 deskewInfo() 需要利用当前点云时间戳之前和之后的IMU/Odom数据进行插值，
        // 如果队列太短，可能相关的IMU/Odom数据还未到位，无法进行有效去畸变。
        if (cloudQueue.size() <= 2)
            return false; // 返回 false，告知主回调函数 cloudHandler 本次不继续执行

        // 3. 取出最早的点云进行处理 (FIFO - 先进先出)
        // 将队列中最旧的一条点云消息移出队列，并赋值给 currentCloudMsg。
        // 使用 std::move 避免不必要的拷贝，提高效率。
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();

        // 4. 点云格式转换：将 ROS 的 sensor_msgs::PointCloud2 转换为 PCL 的 pcl::PointCloud<PointXYZIRT>
        // 根据不同的雷达传感器类型，进行不同的转换操作。
        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
        {
            // 对于 Velodyne 和 Livox 雷达，直接使用 pcl::moveFromROSMsg 转换。
            // laserCloudIn 是一个 pcl::PointCloud<PointXYZIRT>::Ptr 类型的指针。
            // 此操作同样会转移数据所有权，避免拷贝。
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }        
        else if (sensor == SensorType::OUSTER)
        {
            // 对于 Ouster 雷达，需要先转换到一个临时的 Ouster 点云格式(tmpOusterCloudIn)，
            // 然后再手动将其数据转换到标准的 Velodyne 点云格式(laserCloudIn)。
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            // 调整目标点云的大小，使其与原始点云一致
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            // 遍历所有点，进行字段映射
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i]; // 源点 (Ouster)
                auto &dst = laserCloudIn->points[i];     // 目标点 (Velodyne-like)
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity; // 强度信息
                dst.ring = src.ring;           // 线束ID
                // Ouster 点的 't' 字段是纳秒时间戳，将其转换为秒并存储。
                // 这是后续进行运动去畸变的关键。
                dst.time = src.t * 1e-9f;
            }
        }
                
        else if (sensor == SensorType::RS)
        {
            // 对于 Ouster 雷达，需要先转换到一个临时的 Ouster 点云格式(tmpOusterCloudIn)，
            // 然后再手动将其数据转换到标准的 Velodyne 点云格式(laserCloudIn)。
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            // 调整目标点云的大小，使其与原始点云一致
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            // 遍历所有点，进行字段映射
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i]; // 源点 (Ouster)
                auto &dst = laserCloudIn->points[i];     // 目标点 (Velodyne-like)
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity; // 强度信息
                dst.ring = src.ring;           // 线束ID
                // Ouster 点的 't' 字段是纳秒时间戳，将其转换为秒并存储。
                // 这是后续进行运动去畸变的关键。
                dst.time = src.t * 1e-9f;
            }
        }
        else
        {
            // 如果是不支持的传感器类型，报错并关闭节点。
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        // 5. 获取点云的时间信息
        // 保存当前正在处理的点云的消息头，其中包含时间戳等信息。
        cloudHeader = currentCloudMsg.header;
        // timeScanCur: 当前点云帧的起始时间戳（通常是第一个点的采集时间）
        timeScanCur = cloudHeader.stamp.toSec();
        // timeScanEnd: 当前点云帧的结束时间戳。
        // 计算方式：起始时间 + 最后一个点相对于起始点的时间偏移量（存储在点的time字段中）。
        // 这说明了该帧点云不是瞬间采集完成的，而是持续了一段时间（一个扫描周期）。
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        // 6. 检查点云是否是"dense"
        // 一个 dense 点云意味着所有点的坐标都是有效的（不包含 Inf 或 NaN 值）。
        // 非 dense 的点云会包含无效点，这会影响后续的匹配和优化，因此需要提前过滤掉。
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // 7. 检查点云中是否包含 "ring" 通道
        // ring 通道存储了每个点所属的激光雷达线束（扫描线）ID。
        // 这是进行特征提取（如计算曲率）和投影所必需的信息。
        static int ringFlag = 0; // 使用静态变量，只检查一次
        if (ringFlag == 0)
        {
            ringFlag = -1; // 先假设没有找到
            // 遍历点云的所有字段
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1; // 找到了
                    break;
                }
            }
            // 如果最终没找到，报错并关闭节点。
            if (ringFlag == -1)
            {
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }

        // 8. 检查点云中是否包含每个点的时间戳通道 ("time" 或 "t")
        // 每个点的时间戳是进行运动去畸变（Deskew）的关键。
        // 它记录了该点相对于本帧起始时间 timeScanCur 的偏移量。
        static int deskewFlag = 0; // 同样使用静态变量，只检查一次
        if (deskewFlag == 0)
        {
            deskewFlag = -1; // 先假设没有找到
            // 遍历点云的所有字段
            for (auto &field : currentCloudMsg.fields)
            {
                if (field.name == "time" || field.name == "t")
                {                   // 检查字段名
                    deskewFlag = 1; // 找到了
                    break;
                }
            }
            // 如果没找到，发出警告。去畸变功能将被禁用，系统在快速运动时会产生显著漂移。
            if (deskewFlag == -1)
                ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        // 9. 返回 true
        // 所有检查和转换都成功完成，告知 cloudHandler 可以继续执行后续的 deskewInfo() 等步骤。
        return true;
    }

    /**
     * 这个函数利用IMU数据做了两件关键的事：
        提供高频初始姿态： 为整个点云帧提供一个平滑的初始姿态估计 (imuRollInit, imuPitchInit, imuYawInit)。
        计算相对旋转变化： 通过对角速度积分，得到一系列时间点 (imuTime[]) 上相对于积分起点的累积旋转量 (imuRotX[], imuRotY[], imuRotZ[])。
            这将用于补偿点云采集过程中由于旋转运动造成的畸变
     *
     */
    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);

        // make sure IMU data available for the scan
        // 检查Imu可用性，确保imu数据覆盖整个点云扫描周期
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }
        // 处理IMU数据，计算用于去畸变的旋转信息
        imuDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        // 初始化标志位，假设IMU数据不可用
        cloudInfo.imuAvailable = false;

        // 1. 清理过期的IMU数据
        // 移除所有时间戳早于 (当前点云起始时间 - 0.01秒) 的IMU数据。
        // 保留一个小的时间缓冲（0.01s），确保有数据能插值到点云起始时刻。
        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        // 清理后再次检查队列是否为空
        if (imuQueue.empty())
            return;

        // 2. 准备积分：重置积分指针
        imuPointerCur = 0;

        // 3. 遍历当前IMU队列中的所有数据
        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // 3a. 获取当前点云帧的初始姿态估计 (Roll, Pitch, Yaw)
            // 找到时间戳最接近但不晚于点云起始时间 timeScanCur 的那个IMU数据
            // 用它的姿态作为整个点云帧的初始姿态估计（高频IMU姿态比低频里程计更平滑）
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            // 如果当前IMU数据时间远超过点云结束时间（多0.01s缓冲），则停止处理 
            // 这一帧遍历完就break
            if (currentImuTime > timeScanEnd + 0.01)
                break;

            // 3b. 对IMU角速度进行积分，得到相对旋转量
            // 初始化积分数组的第一个元素
            if (imuPointerCur == 0)
            {
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // 从IMU消息中提取角速度值
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // 计算当前IMU时刻与上一次IMU时刻的时间差
            double timeDiff = currentImuTime - imuTime[imuPointerCur - 1];
            // 对角速度进行积分：旋转变化量 = 角速度 * 时间
            // 这里存储的是从积分起点开始累积的相对旋转量（弧度）
            // 计算每一个时刻的姿态角，方便后续查找对应每个点云时间的值
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur - 1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur - 1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur - 1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            // 指针指向下一个位置
            ++imuPointerCur;
        }

        // 循环结束后，指针指向最后一个有效元素的下一个位置，所以需要回退一步
        --imuPointerCur;

        // 检查是否成功积分了有效的IMU数据
        if (imuPointerCur <= 0)
            return;

        // 4. 设置标志位，表明IMU数据可用
        // 现在 cloudInfo 中包含了初始姿态，并且 imuRotX/Y/Z 和 imuTime 数组中存储了
        // 一系列时间戳及其对应的累积相对旋转量。后续可以通过插值得到点云中任意时刻的旋转。
        cloudInfo.imuAvailable = true;
    }

    /**
     *1、 提供优化初始值： 为后续的图优化提供一个良好的初始位姿猜测 (initialGuessX等)，加速并提高优化过程的收敛性和准确性。
        2、计算相对位姿变换： 计算出点云扫描期间机器人坐标系的整体位姿变化 (odomIncreX/Y/Z和未使用的欧拉角增量)。
        这个变换描述了扫描开始和扫描结束两个“瞬间”的位姿差。注意：这与IMU提供的连续旋转积分不同，里程计提供的是一个离散的、整体的变换估计。
        在后续处理中，如果IMU数据不可用，可以用这个整体变换（假设是匀速模型）来近似补偿运动畸变。
     *
     */
    void odomDeskewInfo()
    {
        // 初始化标志位，假设里程计数据不可用
        cloudInfo.odomAvailable = false;

        // 1. 清理过期的里程计数据（逻辑同IMU）
        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        // 检查最早的里程计数据是否晚于点云起始时间，如果是，则没有可用于起始时刻的数据
        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // 2. 寻找点云扫描起始时刻 (timeScanCur) 对应的里程计位姿
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];
            // 遍历队列，找到第一个时间戳 >= timeScanCur 的里程计消息
            // 这个消息的位姿将作为点云帧的初始位姿猜测
            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        // 3. 将初始位姿猜测存入 cloudInfo，用于后续地图优化的初始值
        // 提取位姿中的四元数并转换为欧拉角
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);
        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // 存储初始猜测的位姿 (x, y, z, roll, pitch, yaw)
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw = yaw;

        // 设置标志位，表明至少找到了初始位姿
        cloudInfo.odomAvailable = true;

        // 4. 尝试计算点云扫描期间的整体运动变换（用于去畸变）
        odomDeskewFlag = false; // 先假设无法计算

        // 检查最新的里程计数据时间是否晚于点云结束时间，确保有数据能覆盖结束点
        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        // 寻找点云扫描结束时刻 (timeScanEnd) 对应的里程计位姿
        nav_msgs::Odometry endOdomMsg;
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];
            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        // 5. 一致性检查：比较起始和结束里程计消息的协方差矩阵的第一个值（有时用于存储序列ID）
        // 如果ID不同，说明这两帧里程计数据可能不连续（例如发生了回环优化），此时计算出的增量不可靠，因此放弃。
        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        // 6. 计算从起始位姿到结束位姿的相对变换
        // 将起始位姿转换为变换矩阵
        Eigen::Affine3f transBegin = pcl::getTransformation(
            startOdomMsg.pose.pose.position.x,
            startOdomMsg.pose.pose.position.y,
            startOdomMsg.pose.pose.position.z,
            roll, pitch, yaw);

        // 计算结束位姿的欧拉角
        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        // 将结束位姿转换为变换矩阵
        Eigen::Affine3f transEnd = pcl::getTransformation(
            endOdomMsg.pose.pose.position.x,
            endOdomMsg.pose.pose.position.y,
            endOdomMsg.pose.pose.position.z,
            roll, pitch, yaw);

        // 计算相对变换: transBt = transBegin^{-1} * transEnd
        // 这个变换表示从起始时刻到结束时刻，机器人坐标系发生的变换。
        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        // 7. 从相对变换矩阵中分解出平移增量和欧拉角增量
        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt,
                                          odomIncreX, odomIncreY, odomIncreZ, // 输出平移增量
                                          rollIncre, pitchIncre, yawIncre);   // 输出旋转增量

        // 8. 设置标志位，表明成功计算出了里程计增量，可用于去畸变
        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        // 1. 初始化输出参数，默认旋转量为0
        *rotXCur = 0;
        *rotYCur = 0;
        *rotZCur = 0;

        // 2. 在IMU时间队列中查找包围当前点时间 pointTime 的两个IMU数据点
        // imuPointerCur 指向最新接收到的IMU数据在环形缓冲区中的位置
        int imuPointerFront = 0; // 从缓冲区头部开始搜索
        // 遍历IMU时间队列，找到第一个时间戳 大于 pointTime 的IMU数据索引
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break; // 找到第一个时间晚于点时间的IMU数据，跳出循环
            ++imuPointerFront;
        }

        // 3. 处理两种插值边界情况
        // 情况A: 点的时间 pointTime 晚于找到的IMU时间，或者它是队列中的第一个元素
        // 这意味着点的时间超出了我们拥有的IMU数据范围，或者正好在第一个IMU数据之后
        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            // 无法进行插值，直接使用最近的IMU数据（imuPointerFront）的旋转量
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        }
        // 情况B: 点的时间 pointTime 处于两个IMU数据点之间（正常情况）
        else
        {
            // 3a. 确定用于插值的两个IMU数据点
            int imuPointerBack = imuPointerFront - 1; // 前一个IMU数据点

            // 3b. 计算线性插值的权重系数
            // ratioFront: 当前点时间距离后一个IMU时间点的比例（越近权重越大）
            // ratioBack: 当前点时间距离前一个IMU时间点的比例（越近权重越大）
            double ratioFront = (pointTime - imuTime[imuPointerBack]) /
                                (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) /
                               (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

            // 3c. 对三个轴的旋转量分别进行线性插值
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0;
        *posYCur = 0;
        *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        // 1. 检查去畸变标志和IMU数据可用性
        // deskewFlag 是一个全局标志，可能在别处被设置（例如odomDeskewInfo中）。
        // 如果不需要去畸变 (deskewFlag == -1) 或者IMU数据不可用，则直接返回原始点。
        // 注意：这里只检查了IMU，但实际上里程计数据也可能用于补偿平移。
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        // 2. 计算当前点的绝对时间戳
        // timeScanCur 是整个点云帧的起始时间（第一个点的时间）
        // relTime 是当前点相对于 timeScanCur 的时间偏移量（通常由雷达驱动提供）
        // 两者相加得到当前点的绝对时间 pointTime
        double pointTime = timeScanCur + relTime;

        // 3. 查找旋转补偿量：根据点的时间戳，插值得到该时刻相对于扫描起始时刻的累积旋转
        // rotXCur, rotYCur, rotZCur 将存储通过IMU积分得到的，在 pointTime 时刻，
        // 雷达坐标系相对于 timeScanCur (扫描起始时刻) 的欧拉角。
        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        // 4. 查找平移补偿量：根据点的相对时间，计算该时刻相对于扫描起始时刻的平移
        // posXCur, posYCur, posZCur 将存储通过里程计插值/估算得到的，在 pointTime 时刻，
        // 雷达坐标系相对于 timeScanCur (扫描起始时刻) 的平移量。
        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        // 5. 计算初始变换的逆矩阵 (只计算一次)
        // firstPointFlag 是一个静态变量或成员变量，初始为 true。
        // 对于一帧点云中的第一个点，需要计算一个关键的变换：transStartInverse
        if (firstPointFlag == true)
        {
            // 获取第一个点时刻的位姿变换矩阵 T_start^cur
            // 这个变换描述了从扫描起始时刻坐标系 到 第一个点采集时刻坐标系 的变换
            // 它包含了该时刻的平移 (posXCur, ...) 和旋转 (rotXCur, ...)
            Eigen::Affine3f transStart = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);

            // 计算它的逆矩阵 T_cur^start
            // 这个逆变换描述了如何从 第一个点采集时刻坐标系 变换回 扫描起始时刻坐标系
            transStartInverse = transStart.inverse();

            // 将标志位设为 false，确保一帧点云中只计算一次
            firstPointFlag = false;
        }

        // 6. 计算当前点时刻的位姿变换矩阵
        // 获取当前点时刻的位姿变换矩阵 T_start^cur_i
        // 这个变换描述了从扫描起始时刻坐标系 到 当前第i个点采集时刻坐标系 的变换
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);

        // 7. 计算关键的综合变换矩阵：从当前点时刻 到 扫描起始时刻
        // 计算相对变换：T_cur_i^start = T_cur_i^start = (T_start^cur_i)^{-1}
        // 但这里用了一种更巧妙的方法，利用了之前计算的第一个点的逆变换：
        // transBt = transStartInverse * transFinal
        //
        // 数学推导：
        // 我们已知：
        //   T_start^cur_0  (第一个点时刻的变换，其逆是 transStartInverse)
        //   T_start^cur_i  (当前点时刻的变换, transFinal)
        // 我们想求：
        //   T_cur_i^start = (T_start^cur_i)^{-1}
        // 或者另一种思路，求从当前点坐标系到起始坐标系的变换：
        //   T_cur_i^start = T_cur_0^start * T_cur_i^cur_0
        //
        // 实际上，这里的 transBt 计算的是：
        //   transBt = (T_start^cur_0)^{-1} * T_start^cur_i
        //          = T_cur_0^start * T_start^cur_i
        //          = T_cur_0^cur_i
        // 这得到的变换是 从当前点时刻坐标系 到 第一个点时刻坐标系 的变换！
        // 但注释写着 "transform points to start"，这里似乎是一个逻辑上的小偏差或命名歧义。
        // 实际上，最终目标是把所有点变换到扫描结束时刻，这里可能是一个中间步骤，
        // 或者“start”在这里指的是“运动补偿的参考时刻”，而未必是扫描起始时刻。
        // 结合LIO-SAM的整体思路，它很可能是将所有点变换到扫描结束时刻。
        // 我们需要查看 findRotation 和 findPosition 的具体实现来确认“起始时刻”和“结束时刻”的定义。
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        // 8. 应用变换，对点进行去畸变
        PointType newPoint;
        // 使用变换矩阵 transBt 对原始点坐标进行变换
        // 新的坐标 = R * [x; y; z] + t
        newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y + transBt(0, 2) * point->z + transBt(0, 3);
        newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y + transBt(1, 2) * point->z + transBt(1, 3);
        newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y + transBt(2, 2) * point->z + transBt(2, 3);
        newPoint.intensity = point->intensity;

        // 9. 返回去畸变后的新点
        return newPoint;
    }

    void projectPointCloud()
    {
        // 获取当前帧点云的总点数
        int cloudSize = laserCloudIn->points.size();

        // 开始进行距离图像投影
        for (int i = 0; i < cloudSize; ++i)
        {
            // 1. 创建一个临时点，并复制当前点的基本属性（坐标和强度）
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            // 2. 计算当前点到雷达中心的距离（范围）
            float range = pointDistance(thisPoint);

            // 3. 距离过滤：忽略距离过近或过远的点
            // lidarMinRange 和 lidarMaxRange 是预设的阈值，例如 1.0m 和 100.0m
            // 太近的点可能是雷达自身的噪声，太远的点精度较差且计算量大
            if (range < lidarMinRange || range > lidarMaxRange)
                continue; // 跳过这个点

            // 4. 获取当前点所属的激光雷达线束（ring）ID
            int rowIdn = laserCloudIn->points[i].ring;
            // 检查线束ID是否在有效范围内 [0, N_SCAN-1]
            // N_SCAN 是雷达的总线数，例如 Velodyne VLP-16 是 16
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue; // 无效的线束ID，跳过

            // 5. 降采样：根据 downsampleRate 跳过一些点，减少计算量
            // 例如，如果 downsampleRate=2，则只处理偶数线束（或奇数，取决于实现）的点
            // 这是一种常见的减少点云密度以提高后续处理速度的方法
            if (rowIdn % downsampleRate != 0)
                continue;

            // 6. 计算当前点在水平方向上的列索引 (columnIdn)
            int columnIdn = -1; // 初始化为无效值

            // 处理 Velodyne 和 Ouster 雷达（具有规则的360度水平视场）
            if (sensor == SensorType::VELODYNE || sensor == SensorType::OUSTER)
            {
                // 计算点的水平角度（方位角）：atan2(x, y) * 180/π
                // 注意：这里用的是 atan2(x, y) 而不是 atan2(y, x)，是为了调整角度的零点和对360度的处理
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

                // 计算水平角分辨率：360度 / 水平扫描点数 (Horizon_SCAN)
                static float ang_res_x = 360.0f / float(Horizon_SCAN);

                // 将水平角度映射到列索引
                // 公式: columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2
                // 这个公式的目的是：
                //   (horizonAngle - 90.0): 将坐标系旋转，使得0度对应雷达的前方（Y轴正方向）
                //   除以 ang_res_x: 将角度转换为索引
                //   取负号: 调整索引增加的方向与雷达旋转方向一致
                //   + Horizon_SCAN/2: 将索引中心移动到图像中间
                columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;

                // 处理角度环绕：如果 columnIdn 超过了最大值，则减去一圈（Horizon_SCAN）
                // 因为水平角度是周期性的（0-360度）
                if (columnIdn >= Horizon_SCAN)
                    columnIdn -= Horizon_SCAN;
            }
            // 处理 Livox 雷达（其扫描模式不规则，非360度）
            else if (sensor == SensorType::LIVOX)
            {
                // 对于Livox，不计算水平角，而是为每条线束维护一个独立的列计数器
                // columnIdnCountVec 是一个大小为 N_SCAN 的向量，记录每条线束当前已处理的点数
                columnIdn = columnIdnCountVec[rowIdn];
                // 计数器递增，为下一个点准备
                columnIdnCountVec[rowIdn] += 1;
            }

            // 7. 检查列索引是否在有效范围内 [0, Horizon_SCAN-1]
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            // 8. 检查目标像素是否已被占用（FLT_MAX 表示空）
            // rangeMat 是一个 cv::Mat，存储每个像素点的距离值
            // 这个检查确保了每个像素只保留一个点（最近的点，因为后面来的点如果更近会覆盖？但这里逻辑是跳过，所以实际是保留第一个点）
            // 注意：这里比较的是 != FLT_MAX，意味着如果已经有值了就跳过。所以这里其实是保留第一个投影到该像素的点，而不是最近的点。
            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            // 9. 核心步骤：对该点进行运动去畸变
            // deskewPoint() 函数利用之前准备的 IMU/里程计数据，根据这个点的时间戳 (laserCloudIn->points[i].time)
            // 计算出它在扫描结束时刻的雷达坐标系下的坐标，消除因运动产生的畸变。
            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            // 10. 更新距离图像：将当前点的距离值存入对应的像素
            rangeMat.at<float>(rowIdn, columnIdn) = range;

            // 11. 将去畸变后的点存入全分辨率点云 fullCloud 中
            // fullCloud 是一个一维数组，大小是 N_SCAN * Horizon_SCAN
            // 索引计算：index = columnIdn + rowIdn * Horizon_SCAN
            // 这样存储相当于将二维的距离图像展开成一维数组
            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
            // 注意：fullCloud 中很多位置是空的（用默认值或NaN填充），只有被投影到的位置有有效点。
        }
    }

    /**
     * 将结构化的距离图像 rangeMat 和 fullCloud 转换回非结构化的点云 extractedCloud，
     * 但同时保留其结构信息于 cloudInfo 中。
        为后续的特征提取模块准备数据，并以一种高效、易于访问的方式组织这些数据。
        输出一个无序点云，包含了所有有效的、已经过运动去畸变的点。
     * 
     */
    void cloudExtraction()
    {
        // 初始化计数器，用于记录提取出的有效点的总数量
        int count = 0;

        // 循环遍历所有的激光线束 (0 到 N_SCAN-1)
        for (int i = 0; i < N_SCAN; ++i)
        {
            // 1. 记录当前线束的起始索引（并预留边界）
            // 将当前线束在 extractedCloud 中的起始索引设置为 (count - 1 + 5)
            // 这里 +5 是一个关键操作：它为当前线束的起始位置预留了5个点的空间。
            // 目的是：在后续处理中，当需要查找某个点的前几个邻居时（例如计算曲率），
            // 即使这个点靠近当前线束的开头，也有足够的点数保证不越界。
            // (count - 1) 是因为count指向的是下一个要存储的位置。
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            // 2. 遍历当前线束的所有水平列 (0 到 Horizon_SCAN-1)
            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                // 检查当前像素是否包含有效点（FLT_MAX 表示空像素）
                if (rangeMat.at<float>(i, j) != FLT_MAX)
                {
                    // 2a. 记录该点的列索引
                    // 将当前点的列索引 j 存入 cloudInfo.pointColInd[count]
                    // 目的：后续特征提取或匹配时，可以快速知道这个点来自距离图像的哪一列，
                    //       从而方便地找到它在水平方向上的邻居。
                    cloudInfo.pointColInd[count] = j;

                    // 2b. 记录该点的距离值
                    // 将当前像素的距离值（来自 rangeMat）存入 cloudInfo.pointRange[count]
                    // 目的：后续计算点的曲率（ curvature = |r1 + r3 - 2*r2| / (r2)^2 ）时，
                    //       可以直接使用这个预存的距离值，无需重新计算点距，提高效率。
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i, j);

                    // 2c. 提取点云
                    // 根据行列索引 (i, j) 计算在一维数组 fullCloud 中的位置：index = j + i * Horizon_SCAN
                    // 将该点的数据（已经过运动去畸变）从 fullCloud 中取出，并加入到 extractedCloud 点云中。
                    // extractedCloud 是一个无序的点云集合，包含了所有有效的、去畸变后的点。
                    extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);


                    // 2d. 增加有效点计数器
                    ++count;
                }
            }

            // 3. 记录当前线束的结束索引（并预留边界）
            // 将当前线束在 extractedCloud 中的结束索引设置为 (count - 1 - 5)
            // 这里 -5 与上面的 +5 对称：它为当前线束的结束位置预留了5个点的空间。
            // 目的：同样是为了保证在查找某个点的后几个邻居时，即使这个点靠近当前线束的末尾，也不会越界。
            cloudInfo.endRingIndex[i] = count - 1 - 5;
        }
    }

    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        // ROS_INFO("Extracted cloud size: %ld", extractedCloud->size());
        publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);

    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rs_lidar_deskew");

    ImageProjection IP;

    ROS_INFO("RS Lidar Deskew Node started successfully");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
