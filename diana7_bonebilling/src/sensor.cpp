#include "DianaAPIDef.h"
#include "DianaAPI.h"
#include <cstring>
#include <iostream>
#include <sri/ftsensor.hpp>
#include <sri/commethernet.hpp>
#include <kdl/frames.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "yaml-cpp/yaml.h"
#include "Iir.h"

using namespace SRI;
#define JOINT_NUM 7
bool isRunning = true;
double wrench[6] = {0.0};
const char *strIpAddress = "192.168.100.75";
double enable_wrench[6] = {0};
// double zero_offset[6] = {-3.0987, -1.25601, 11.7307, -0.000, 0.000, 0.000};
// double zero_offset[6] = {-3.0987, -1.25601, 14.1741, -0.000, 0.000, 0.000};
bool isFtSensor = false;
bool noError = true;
double Distance_min = 100.0;
//std::string yaml_path = "normal.yaml";
std::string yaml_path = "/home/wx/Documents/Diana/src/diana7_bonebilling/src/normal.yaml";
YAML::Node yaml_node;
// auto node;
rclcpp::Node::SharedPtr node;

void rtDataHandler(std::vector<RTData<float>> &rtData)
{
    isFtSensor = true;

    // rclcpp::Time current_time =node->get_clock()->now();
    // rclcpp::Duration duration = rclcpp::Duration::from_seconds(global_publish_frequency);

    // std::cout << "[" << i << "] RT Data is ->  ";
    for (int i = 0; i < rtData.size(); i++)
    {
        for (int j = 0; j < 6; j++)
        {
            // std::cout << "Ch " << j << ": " << rtData[i][j] << "\t";
            enable_wrench[j] = rtData[i][j];
        }
    }
}

void logRobotState(StrRobotStateInfo *pinfo, const char *strIpAddress)
{
}
void errorControl(int e, const char *strIpAddress)
{
    strIpAddress = "192.168.100.75";
    const char *strError = formatError(e); // 该函数后面会介绍
    printf("error code (%d):%s\n", e, strError);

    noError = false;
}
KDL::Frame calplat(KDL::Vector normal, KDL::Vector p)
{
    KDL::Vector gx{1, 0, 0};

    KDL::Vector x, y;
    KDL::Vector z = normal;
    z.Normalize();
    y = z * gx;
    double norm = y.Normalize();
    if (norm < 0.1)
    {
        KDL::Vector gy{0, 1, 0};
        x = gy * z;
        x.Normalize();
        y = z * x;
        y.Normalize();
    }
    else
    {
        x = y * z;
        x.Normalize();
    }

    KDL::Rotation rot(x, y, z);
    KDL::Frame frame(rot, p);
    return frame; // 相对于基坐标系的位姿
}

KDL::Wrench getTheoryWrench(double *pose_tcp, double mass, KDL::Vector center_of_mass_position)
{
    //! The output of the function is the theory wrench
    KDL::Vector translation(0.0, 0.0, mass * -9.81);
    KDL::Vector axis = KDL::Vector(pose_tcp[3], pose_tcp[4], pose_tcp[5]);
    double norm = axis.Normalize();
    KDL::Frame TCP_base = KDL::Frame(KDL::Rotation::Rot(axis, norm), KDL::Vector(pose_tcp[0], pose_tcp[1], pose_tcp[2]));
    // KDL::Rotation f_TCP = KDL::Rotation::RPY(M_PI, 0, 0);
    KDL::Rotation f_TCP = KDL::Rotation::RPY(M_PI, 0,-M_PI/4);
    KDL::Frame f_base;
    f_base.p = TCP_base.p;
    f_base.M = TCP_base.M * f_TCP;
    f_base.M.SetInverse();
    KDL::Vector rotated_translation = f_base.M * translation;
    KDL::Rotation cross_mass = KDL::Rotation(0, -center_of_mass_position[2], center_of_mass_position[1],
                                             center_of_mass_position[2], 0, -center_of_mass_position[0],
                                             -center_of_mass_position[1], center_of_mass_position[0], 0);
    KDL::Wrench wrench_;
    wrench_.force = rotated_translation;
    wrench_.torque = cross_mass * wrench_.force;
    // std::cout << "Theory_wrench: " << wrench_.force.data[0] << "," << wrench_.force.data[1] << "," << wrench_.force.data[2] << "," << wrench_.torque.data[0] << "," << wrench_.torque.data[1] << "," << wrench_.torque.data[2] << std::endl;
    return wrench_;
}
void getZeroOffset(double *pose_tcp, double *wrench, double mass, KDL::Vector center_of_mass_position, double *ZeroOffset)
{

    KDL::Wrench Theory_wrench = getTheoryWrench(pose_tcp, mass, center_of_mass_position);
    ZeroOffset[0] = wrench[0] - Theory_wrench.force.data[0];
    ZeroOffset[1] = wrench[1] - Theory_wrench.force.data[1];
    ZeroOffset[2] = wrench[2] - Theory_wrench.force.data[2];
    ZeroOffset[3] = wrench[3] - Theory_wrench.torque.data[0];
    ZeroOffset[4] = wrench[4] - Theory_wrench.torque.data[1];
    ZeroOffset[5] = wrench[5] - Theory_wrench.torque.data[2];
}
KDL::Wrench gravityCompensation(double *pose_tcp, double *wrench, double *Zero_offset, double mass, KDL::Vector center_of_mass_position)
{
    //! The feedback of function is the exit wrench
    // Define the gravity vector
    // KDL::Vector gravity(0.0, 0.0, -9.81);
    // // Define the mass of the end-effector
    // double mass = 0.249072;
    KDL::Vector translation(0.0, 0.0, mass * -9.81);
    // Define the mass center of the end-effector
    // KDL::Vector center_of_mass_position(0.0, 0.0, 0.0366358);
    KDL::Wrench Theory_wrench = getTheoryWrench(pose_tcp, mass, center_of_mass_position);
    KDL::Wrench wrench_;
    wrench_.force.data[0] = wrench[0] - Theory_wrench.force.data[0] - Zero_offset[0];
    wrench_.force.data[1] = wrench[1] - Theory_wrench.force.data[1] - Zero_offset[1];
    wrench_.force.data[2] = wrench[2] - Theory_wrench.force.data[2] - Zero_offset[2];
    wrench_.torque.data[0] = wrench[3] - Theory_wrench.torque.data[0] - Zero_offset[3];
    wrench_.torque.data[1] = wrench[4] - Theory_wrench.torque.data[1] - Zero_offset[4];
    wrench_.torque.data[2] = wrench[5] - Theory_wrench.torque.data[2] - Zero_offset[5];
    return wrench_;
}
void wait_move(const char *strIpAddress)
{
    usleep(20000);
    while (true)
    {
        const char state = getRobotState(strIpAddress);
        if (state != 0)
        {
            break;
        }
        else
        {
            usleep(1000);
        }
    }
    stop(strIpAddress);
}
void signalHandler(int signo)
{
    if (signo == SIGINT)
    {
        std::cout << "\033[1;31m"
                  << "[!!SIGNAL!!]"
                  << "INTERRUPT by CTRL-C"
                  << "\033[0m" << std::endl;

        isRunning = false;
        exit(0);
    }
}
template <typename T>
int sign(T val) {
    return (val > T(0)) - (val < T(0));
}

struct Point {
    double x;
    double y;
};

// 计算两点之间的距离
double distance(const std::vector<double>& p1, const std::vector<double>& p2) {
    return std::sqrt(std::pow(p2[0] - p1[0], 2) + std::pow(p2[1] - p1[1], 2));
}

// 计算点到直线的距离
double distanceToLine(const std::vector<double>& p1, const std::vector<double>& p2, const std::vector<double>& p) {
    
    double dx = p2[0] - p1[0];
    double dy = p2[1] - p1[1];
    double u = ((p[0] - p1[0]) * dx + (p[1] - p1[1]) * dy) / (dx * dx + dy * dy);
    double closest_x = p1[0] + u * dx;
    double closest_y = p1[1] + u * dy;
    return distance(p, {closest_x, closest_y});
}

// 计算多边形到点的最短距离
double shortestDistanceToPolygon(const std::vector<std::vector<double>>& polygon, const std::vector<double>& p) {
    double min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < polygon.size(); ++i) {
        // 计算多边形的每条边
        const auto p1 = polygon[i];
        const auto p2 = polygon[(i + 1) % polygon.size()];
        
        // 计算边到点的距离
        double dist = distanceToLine(p1, p2, p);
        
        // 更新最小距离
        if (dist < min_distance) {
            min_distance = dist;
        }
    }

    return min_distance;
}

//判断平面上一点是否在多边形内
bool is_in_2d_polygon(std::vector<double> point, std::vector<std::vector<double>> vertices) {
    double px = point[0];
    double py = point[1];
    double angle_sum = 0;
    // double dis_min =0;
    double dis = 0;

    int size = vertices.size();
    
    if (size < 3) {
        throw std::invalid_argument("len of vertices < 3");
    }
    int j = size - 1;
    for (int i = 0; i < size; i++) {
        double sx = vertices[i][0];
        double sy = vertices[i][1];
        double tx = vertices[j][0];
        double ty = vertices[j][1];


        double k = (sy - ty) / (sx - tx + 0.000000000001);
        double b = sy - k * sx;
        
        dis = std::fabs(k * px - 1 * py + b) / std::sqrt(k * k + 1);
        // dis_min = (dis_min < dis )? dis_min : dis;
        if (dis < 0.002) {
            if ((sx <= px && px <= tx) || (tx <= px && px <= sx)) {
                return true;
            }
        }

        double angle = std::atan2(sy - py, sx - px) - std::atan2(ty - py, tx - px);
        
        if (angle >= M_PI) {
            angle -= M_PI * 2;
        } else if (angle <= -M_PI) {
            angle += M_PI * 2;
        }
         
        angle_sum += angle;
        j = i;
    }
    
    return std::fabs(angle_sum - M_PI * 2) < 1e-4;
    
}
//判断空间上一点是否在多边形内
bool is_in_3d_polygon(std::vector<double> point, KDL::Vector normal, std::vector<geometry_msgs::msg::Point32>& points) {
    std::vector<std::vector<double>> local_v;
   
    for (const auto& ppp : points) {
        local_v.push_back({ppp.x, ppp.y, ppp.z});
    }
    std::vector<double> local_p = point;

    double na = normal.x();
    double nb = normal.y();
    double nc = normal.z();
    double d = -(na * local_v[0][0] + nb * local_v[0][1] + nc * local_v[0][2]);
    double distance = std::fabs(na * local_p[0] + nb * local_p[1] + nc * local_p[2] + d) \
                      / (std::sqrt(na * na + nb * nb + nc * nc));

    // if (distance > 0.001) {
    //     return false;
    // }

    int index = 2;
    if (normal.x() != 0) {
        index = 0;
    } else if (normal.y() != 0) {
        index = 1;
    } else if (normal.z() != 0) {
        index = 2;
    } else {
        throw std::invalid_argument("All elem in normal is zero");
    }

    local_p.erase(local_p.begin() + index);
    for (int i = 0; i < local_v.size() - 1; i++) {
        local_v[i].erase(local_v[i].begin() + index);
    }

    return is_in_2d_polygon(local_p, local_v);
}
//共平面三点求法向量
std::vector<double> get_normal(std::vector<double> point_a, std::vector<double> point_b, std::vector<double> point_c) {
    std::vector<double> a(3);
    std::vector<double> b(3);

    a[0] = point_a[0] - point_b[0];
    a[1] = point_a[1] - point_b[1];
    a[2] = point_a[2] - point_b[2];

    b[0] = point_a[0] - point_c[0];
    b[1] = point_a[1] - point_c[1];
    b[2] = point_a[2] - point_c[2];

    std::vector<double> normal(3);
    normal[0] = a[1] * b[2] - a[2] * b[1];
    normal[1] = a[2] * b[0] - a[0] * b[2];
    normal[2] = a[0] * b[1] - a[1] * b[0];

    if (normal[0] == 0 && normal[1] == 0 && normal[2] == 0) {
        return std::vector<double>();
    } else {
        return normal;
    }
}

int main(int argc, char const *argv[])
{
    // yaml初始化
    yaml_node = YAML::LoadFile(yaml_path);
    // 信号处理
    if (signal(SIGINT, signalHandler) == SIG_ERR)
    {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }
    // ROS2初始化
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("joint_state_publisher");
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    auto wrench_z = node->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_z", 10);
    auto wrench_xy = node->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_xy", 10);
    auto plane_xy = node->create_publisher<geometry_msgs::msg::PolygonStamped>("plane_xy", 10);
    rclcpp::WallRate loop_rate(500); // 1 Hz publishing rate
    geometry_msgs::msg::WrenchStamped wrench_msg_z;
    geometry_msgs::msg::WrenchStamped wrench_msg_xy;
    geometry_msgs::msg::PolygonStamped plane_msg_xy;
    plane_msg_xy.header.stamp = node->get_clock()->now();
    wrench_msg_xy.header.stamp = node->get_clock()->now();
    wrench_msg_xy.header.frame_id = "link_7";
    wrench_msg_z.header.stamp = node->get_clock()->now();
    wrench_msg_z.header.frame_id = "link_7";
    // 低通滤波器
    std::array<Iir::Butterworth::LowPass<2>, 3> filter_array{};
    const float samplingrate = 200;    // Hz
    const float cutoff_frequency = 5; // Hz
    filter_array[0].setup(2, samplingrate, cutoff_frequency);
    filter_array[1].setup(2, samplingrate, cutoff_frequency);
    filter_array[2].setup(2, samplingrate, cutoff_frequency);
    // 传感器初始化
    SRI::CommEthernet *ce = new SRI::CommEthernet("192.168.0.108", 4008);
    SRI::FTSensor sensor(ce);
    double x, y, z, mx, my, mz;
    auto rtDataValid = sensor.getRealTimeDataValid();
    auto rtMode = sensor.getRealTimeDataMode();
    // sensor.startRealTimeDataRepeatedly<float>(&rtDataHandler, rtMode, rtDataValid);
    srv_net_st *pinfo = new srv_net_st();
    memset(pinfo->SrvIp, 0x00, sizeof(pinfo->SrvIp));
    memcpy(pinfo->SrvIp, "192.168.100.75", strlen("192.168.100.75"));
    pinfo->LocHeartbeatPort = 0;
    pinfo->LocRobotStatePort = 0;
    pinfo->LocSrvPort = 0;
    int ret = initSrv(errorControl, logRobotState, pinfo);
    if (ret < 0)
    {
        printf("192.168.100.75 initSrv failed! Return value = %d\n", ret);
    }
    if (pinfo)
    {
        delete pinfo;
        pinfo = nullptr;
    }
    //
    releaseBrake(strIpAddress);

    const double PI = 3.141592653;
    double joints[7] = {0, 0, 0, PI / 2, 0, -PI / 2, 0}; // 以 7 轴机器人为例

    // double joints[JOINT_NUM] = {0.0};
    double poses[6] = {0.0};
    // 初始点位
    moveJToTarget(joints, 0.5, 0.5, 0, 0, 0, strIpAddress);
    wait_move(strIpAddress);
    ret = getTcpPos(poses, strIpAddress);
    if (ret < 0)
    {
        printf("getJointPos failed! Return value = %d\n", ret);
    }
    else
    {
        std::cout << "joints: " << joints[0] << " " << joints[1] << " " << joints[2] << " " << joints[3] << " " << joints[4] << " " << joints[5] << " " << joints[6] << std::endl;
    }

    // 上电采集力信息数据

    int sum = 1000;
    for (int i = 0; i < sum; i++)
    {
        auto rtData = sensor.getRealTimeDataOnce<float>(rtMode, rtDataValid);

        for (int j = 0; j < rtData.size(); j++)
        {
            enable_wrench[0] += rtData[j][0];
            enable_wrench[1] += rtData[j][1];
            enable_wrench[2] += rtData[j][2];
            enable_wrench[3] += rtData[j][3];
            enable_wrench[4] += rtData[j][4];
            enable_wrench[5] += rtData[j][5];
        }
    }
    enable_wrench[0] /= sum;
    enable_wrench[1] /= sum;
    enable_wrench[2] /= sum;
    enable_wrench[3] /= sum;
    enable_wrench[4] /= sum;
    enable_wrench[5] /= sum;
    std::cout << "enable_wrench: " << enable_wrench[0] << "," << enable_wrench[1] << "," << enable_wrench[2] << "," << enable_wrench[3] << "," << enable_wrench[4] << "," << enable_wrench[5] << std::endl;
    //质量与质心
    double mass = 1.52;
    KDL::Vector center_of_mass_position(-0.0307868, -0.00347722, 0.0535391);
    double Zero_offset[6] = {0.0};
    getZeroOffset(poses, enable_wrench, mass, center_of_mass_position, Zero_offset);
    std::cout << "Zero_offset: " << Zero_offset[0] << "," << Zero_offset[1] << "," << Zero_offset[2] << "," << Zero_offset[3] << "," << Zero_offset[4] << "," << Zero_offset[5] << std::endl;

    // 向量平面确定
    KDL::Vector p(poses[0], poses[1], poses[2]);
    // normal.yaml数据读取
    KDL::Vector normal(0, 1, 1);
    if (yaml_node["normal"])
    {
        std::vector<double> normal_data = yaml_node["normal"].as<std::vector<double>>();
        normal = KDL::Vector(normal_data[0], normal_data[1], normal_data[2]);
    }
    else
    {
        std::cout << "normal.yaml文件中没有normal数据,默认为0,1,1" << std::endl;
    }

    KDL::Frame frame = calplat(normal, p);
    KDL::Vector r = frame.M.GetRot();
    // KDL::Wrench wrench;
    double pose_target[6] = {frame.p.x(), frame.p.y(), frame.p.z(), r.x(), r.y(), r.z()};
    double vel = 0.1;
    double acc = 0.1;
    double radius = 0.0;
    int zv_shaper_order = 0;
    double zv_shaper_frequency = 0;
    double zv_shaper_damping_ratio = 0;
    ret = moveLToPose(pose_target, vel, acc, nullptr, zv_shaper_order, zv_shaper_frequency, zv_shaper_damping_ratio, strIpAddress);
    wait_move(strIpAddress);
    memset(enable_wrench, 0, sizeof(enable_wrench));
    for (int i = 0; i < sum; i++)
    {
        auto rtData = sensor.getRealTimeDataOnce<float>(rtMode, rtDataValid);

        for (int j = 0; j < rtData.size(); j++)
        {
            enable_wrench[0] += rtData[j][0];
            enable_wrench[1] += rtData[j][1];
            enable_wrench[2] += rtData[j][2];
            enable_wrench[3] += rtData[j][3];
            enable_wrench[4] += rtData[j][4];
            enable_wrench[5] += rtData[j][5];
        }
    }
    enable_wrench[0] /= sum;
    enable_wrench[1] /= sum;
    enable_wrench[2] /= sum;
    enable_wrench[3] /= sum;
    enable_wrench[4] /= sum;
    enable_wrench[5] /= sum;
    getTcpPos(poses, strIpAddress);
    KDL::Wrench wrench_compensation = gravityCompensation(poses, enable_wrench, Zero_offset, mass, center_of_mass_position);
    std::cout << "wrench_compensation: " << wrench_compensation.force.data[0] << "," << wrench_compensation.force.data[1] << "," << wrench_compensation.force.data[2] << "," << wrench_compensation.torque.data[0] << "," << wrench_compensation.torque.data[1] << "," << wrench_compensation.torque.data[2] << std::endl;
    double B = 20000.0;
    double B_ = B;
    KDL::Rotation f_tcp = KDL::Rotation::RPY(M_PI, 0, -M_PI/4);
    // getTcpPos(poses, strIpAddress);
    KDL::Vector axis = KDL::Vector(poses[3], poses[4], poses[5]);
    double norm = axis.Normalize();
    double rot_poses[6] = {0.0};
    KDL::Frame TCP_base = KDL::Frame(KDL::Rotation::Rot(axis, norm), KDL::Vector(poses[0], poses[1], poses[2]));
    KDL::Vector poses_origin = KDL::Vector(poses[0], poses[1], poses[2]);
    KDL::Frame p_TCP = TCP_base.Inverse();
    sensor.startRealTimeDataRepeatedly<float>(&rtDataHandler, rtMode, rtDataValid);
    // 任意四个点确定一个平面
    double plane_length = 0.2;
    KDL::Frame p1 = TCP_base * KDL::Frame(KDL::Vector(plane_length, plane_length, 0));
    KDL::Frame p2 = TCP_base * KDL::Frame(KDL::Vector(-plane_length, plane_length, 0));
    KDL::Frame p3 = TCP_base * KDL::Frame(KDL::Vector(-plane_length, -plane_length, 0));
    KDL::Frame p4 = TCP_base * KDL::Frame(KDL::Vector(plane_length, -plane_length, 0));
    geometry_msgs::msg::Point32 point1;
    geometry_msgs::msg::Point32 point2;
    geometry_msgs::msg::Point32 point3;
    geometry_msgs::msg::Point32 point4;

    point1.x = p1.p.x();
    point1.y = p1.p.y();
    point1.z = p1.p.z();
    point2.x = p2.p.x();
    point2.y = p2.p.y();
    point2.z = p2.p.z();
    point3.x = p3.p.x();
    point3.y = p3.p.y();
    point3.z = p3.p.z();
    point4.x = p4.p.x();
    point4.y = p4.p.y();
    point4.z = p4.p.z();
    plane_msg_xy.header.frame_id = "base"; // 设置坐标系为 "map"

    // 添加多边形顶点
    //plane_msg_xy.polygon.points.resize(4);
    plane_msg_xy.polygon.points.push_back(point1);
    plane_msg_xy.polygon.points.push_back(point2);
    plane_msg_xy.polygon.points.push_back(point3);
    plane_msg_xy.polygon.points.push_back(point4);
    //上一时刻的力和力矩
    KDL::Wrench last_wrench;
    last_wrench.force.x(0);
    last_wrench.force.y(0);
    last_wrench.force.z(0);
    KDL::Wrench wrench_d;
    while (isRunning)
    {
        if (getRobotState(strIpAddress) == 6)
        {
            noError = false;
        }
        // ! ros2 publish
        auto joint_state_ = sensor_msgs::msg::JointState();

        // Set joint names
        joint_state_.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};

        // Set joint positions
        // joint_state_.position = {0.0, 0.0, 0.0,0,0,0,0};
        double joint_pos[7] = {0.0};
        getJointPos(joint_pos, strIpAddress);
        joint_state_.position = {joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5], joint_pos[6]};

        // Set header timestamp
        joint_state_.header.stamp = node->get_clock()->now();

        publisher->publish(joint_state_);
        plane_msg_xy.header.stamp = node->get_clock()->now();
        // plane_msg_xy.polygon.points.push_back(point1);
        // plane_msg_xy.polygon.points.push_back(point2);
        // plane_msg_xy.polygon.points.push_back(point3);
        // plane_msg_xy.polygon.points.push_back(point4);
        plane_xy->publish(plane_msg_xy);
        
        if (noError)
        {
            
            // wrench_compensation.force.y(1);
            std::cout << "enable_wrench: " << enable_wrench[0] << "," << enable_wrench[1] << "," << enable_wrench[2] << "," << enable_wrench[3] << "," << enable_wrench[4] << "," << enable_wrench[5] << std::endl;
   
            wrench_compensation = gravityCompensation(poses, enable_wrench, Zero_offset, mass, center_of_mass_position);
            std::cout << "wrench_compensation:" << wrench_compensation.force.data[0] <<","<< wrench_compensation.force.data[1]<<","<< wrench_compensation.force.data[2]<<","<<wrench_compensation.torque.data[0]<<","<<wrench_compensation.torque.data[1]<<","<<wrench_compensation.torque.data[2]<< std::endl;
            
            wrench_compensation.force.x(filter_array[0].filter(wrench_compensation.force.x()));
            wrench_compensation.force.y(filter_array[1].filter(wrench_compensation.force.y()));
            wrench_compensation.force.z(filter_array[2].filter(wrench_compensation.force.z()));
            wrench_d.force.x((wrench_compensation.force.x() - last_wrench.force.x())*200);
            wrench_d.force.y((wrench_compensation.force.y() - last_wrench.force.y())*200);
            wrench_d.force.z((wrench_compensation.force.z() - last_wrench.force.z())*200);
            last_wrench = wrench_compensation;
            // if(abs(wrench_d.force.x()) > 300)
            // {
            //     wrench_compensation.force.x(0*sign(wrench_compensation.force.x()));
            //     std::cerr<<"x方向发生碰撞: "<<1*sign(wrench_compensation.force.x())<<"last_wrench:"<<last_wrench.force.x()<<std::endl;
            //     getTcpPos(poses, strIpAddress);
            // }
            // if(abs(wrench_d.force.y()) > 300)
            // {
            //     wrench_compensation.force.y(0*sign(wrench_compensation.force.y()));
            //     std::cerr<<"y方向发生碰撞"  << 0*sign(wrench_compensation.force.y()) << std::endl;
            //     getTcpPos(poses, strIpAddress);
            // }
            // if(abs(wrench_d.force.z() )> 300)
            // {
            //     wrench_compensation.force.z(0*sign(wrench_compensation.force.z()));
            //     std::cerr<<"z方向发生碰撞"  << std::endl;
            //     // getTcpPos(poses, strIpAddress);
            // }
            
            
            //死区
            if (abs(wrench_compensation.force.x()) < 1)
            {
                wrench_compensation.force.x(0);
            }
            if (abs(wrench_compensation.force.y()) < 1)
            {
                wrench_compensation.force.y(0);
            }
            // //限幅
            // if (abs(wrench_compensation.force.x()) > 5)
            // {
            //     wrench_compensation.force.y(5*wrench_compensation.force.y()/wrench_compensation.force.x());
            //     wrench_compensation.force.x(5*sign(wrench_compensation.force.x()));              
            //     std::cerr<<"x方向发生限幅: "<<5*sign(wrench_compensation.force.x())<<std::endl;
            // }
            // if (abs(wrench_compensation.force.y()) > 5)
            // {
            //     wrench_compensation.force.x(5*(wrench_compensation.force.x()/wrench_compensation.force.y()));
            //     wrench_compensation.force.y(5*sign(wrench_compensation.force.y()));
            //     std::cerr<<"y方向发生限幅: "<<5*sign(wrench_compensation.force.y())<<std::endl;
            // }
            KDL::Wrench wrench_temp = KDL::Rotation::RPY(PI, 0, 0) * wrench_compensation;
            // wrench_compensation.force.x(0);
            // wrench_compensation.force.y(0);
            wrench_msg_z.header.stamp = node->get_clock()->now();

            wrench_msg_z.wrench.force.z = wrench_temp.force.z();
            wrench_msg_xy.header.stamp = node->get_clock()->now();
            wrench_msg_xy.wrench.force.x = wrench_temp.force.x();
            wrench_msg_xy.wrench.force.y = wrench_temp.force.y();

            wrench_z->publish(wrench_msg_z);
            wrench_xy->publish(wrench_msg_xy);

            wrench_compensation.force.z(0);
            wrench_compensation.torque.x(0);
            wrench_compensation.torque.y(0);
            wrench_compensation.torque.z(0);
            // if(wrench_compensation.torque.z() > 1.5)
            // {
            //     wrench_compensation.force.x(0);
            //     wrench_compensation.force.y(0);
            // }else
            // {
            //     wrench_compensation.torque.z(0);
            // };
            KDL::Wrench wrench_base = TCP_base.M * f_tcp * wrench_compensation;
            
            KDL::Vector point_TCP = p_TCP.M*KDL::Vector(poses[0], poses[1], poses[2]) + p_TCP.p;
            // KDL::Vector point_TCP = TCP_base.M.Inverse()*KDL::Vector(poses[0], poses[1], poses[2]);
            // std::cout << "p_TCP.p " << p_TCP.p.x() <<std::endl;
            // std::cout << "p_TCP.p " << p_TCP.p.y() <<std::endl;
            // std::cout << "point_TCP_x " << point_TCP.x() <<std::endl;
            // std::cout << "point_TCP_y " << point_TCP.y() <<std::endl;
            std::vector<double> point;
            point.resize(2);
            std::vector<std::vector<double>> vertices ={{plane_length,plane_length},{-plane_length,plane_length},{-plane_length,-plane_length},{plane_length,-plane_length}};
            point[0] = point_TCP.x();
            point[1] = point_TCP.y();

           
    
            // 计算多边形到点的最短距离
            Distance_min = shortestDistanceToPolygon(vertices, point);
            //获取实际位姿
            // getTcpPos(poses, strIpAddress);

            KDL::Vector poses_current = KDL::Vector(poses[0], poses[1], poses[2]);
            poses[0] += wrench_base.force.x() / B_;
            poses[1] += wrench_base.force.y() / B_;
            poses[2] += wrench_base.force.z() / B_;
            KDL::Vector poses_next = KDL::Vector(poses[0], poses[1], poses[2]);
            KDL::Vector point_TCP_next = p_TCP.M*KDL::Vector(poses[0], poses[1], poses[2]) + p_TCP.p;
            std::vector<double> point_next = {point_TCP_next.x(),point_TCP_next.y()};
            
            double Distance_start_damping = 0.1;
            if(!is_in_2d_polygon(point,vertices))
            
            {
                std::cerr<<"在多边形外"  << std::endl;
                
                
               
                
                
                if (!is_in_2d_polygon(point_next,vertices))
                {
                    poses[0] = poses_current.x();
                    poses[1] = poses_current.y();
                    poses[2] = poses_current.z();
                }else
                {
                    // poses[0] += wrench_base.force.x() / B;
                    // poses[1] += wrench_base.force.y() / B;
                    // poses[2] += wrench_base.force.z() / B;
                }
                
                

            }else
            {
                std::cerr<<"在多边形内"  << std::endl;
                std::cout << "Distance_min: " << Distance_min << std::endl;
                if(Distance_min < Distance_start_damping)
                {
                    
                    // B_ = exp(1e1*( Distance_start_damping - Distance_min))*B;
                    B_ = B + ( Distance_start_damping - Distance_min)*1e6*6;
                    // poses[0] += poses_current.x() / B_;
                    // poses[1] += poses_current.y() / B_;
                    // poses[2] += poses_current.z() / B_;
                }else{

                    B_ = B;
                }
                
                // if((poses_current-poses_next).Norm() > 1e-3)
                
            }
            
            // if (!is_in_2d_polygon(point_last,vertices))
            //     {
            //         poses[0] = poses_last.x();
            //         poses[1] = poses_last.y();
            //         poses[2] = poses_last.z();
            //     }
                       
            // getTcpPos(rot_poses, strIpAddress);
            axis = KDL::Vector(poses[3], poses[4], poses[5]);
            auto n = normal;
            n.Normalize();
            double norm = axis.Normalize();
            double angle = -wrench_compensation.torque.z() / 1000.0;
            auto delta_R = KDL::Rotation::Rot(n, angle);
            auto m = delta_R * KDL::Rotation::Rot(axis, norm);
            TCP_base = KDL::Frame(m, KDL::Vector(poses[0], poses[1], poses[2]));
            KDL::Vector r = TCP_base.M.GetRot();
            poses[3] = r.x();
            poses[4] = r.y();
            poses[5] = r.z();
            // std::cout << "poses: " << poses[0] << "," << poses[1] << "," << poses[2] << std::endl;
            // std::cout<<"Wrench_base: "<<wrench_base.force.x()<<","<<wrench_base.force.y()<<","<<wrench_base.force.z()<<","<<wrench_base.torque.x()<<","<<wrench_base.torque.y()<<","<<wrench_base.torque.z()<<std::endl;
            // poses[0] += poses_current.x() / B_;
            // poses[1] += poses_current.y() / B_;
            // poses[2] += poses_current.z() / B_;
            servoL(poses, 0.01, 0.1, 300, 1.0, nullptr, strIpAddress);
            
            // std::cout<<"poses: "<<poses[0]<<","<<poses[1]<<","<<poses[2]<<","<<poses[3]<<","<<poses[4]<<","<<poses[5]<<std::endl;
            // std::cout << "wrench z: " << std::fixed << wrench_compensation.torque.z() << " ; Angle: " << angle << std::endl;
            usleep(1000);
            
        }
        else
        {
            // while(getRobotState(strIpAddress) == 6)
            //     cleanErrorInfo(strIpAddress);
            // std::cout << "ClearErrorInfo!!" << std::endl;
            noError = true;

            getTcpPos(poses, strIpAddress);

            // usleep(10000);

            cleanErrorInfo(strIpAddress);
            setLastError(0, strIpAddress);
            // std::cout << "Clear Error!!" << std::endl;

            usleep(1000);
        }
    }

    destroySrv(strIpAddress);
    return 0;
}
