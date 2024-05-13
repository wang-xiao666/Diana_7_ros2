#include "DianaAPIDef.h"
#include "DianaAPI.h"
#include <cstring>
#include <iostream>
#include <sri/ftsensor.hpp>
#include <sri/commethernet.hpp>
#include <sri/SixDForceTool.hpp>
#include <kdl/frames.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "yaml-cpp/yaml.h"
#include "Iir.h"
#include <Eigen/Dense>

using namespace SRI;
#define JOINT_NUM 7
bool isRunning = true;
double wrench[6] = {0.0};
const char *strIpAddress = "192.168.100.75";
double enable_wrench[6] = {0};

bool isFtSensor = false;
bool noError = true;
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
    KDL::Rotation f_TCP = KDL::Rotation::RPY(M_PI, 0, 0);
    //KDL::Rotation f_TCP = KDL::Rotation::RPY(M_PI, 0, M_PI/4);
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

std::vector<double> ATIGravityCompensationCalParam(Eigen::MatrixXd M)
{
    auto F0 = Eigen::Vector3d(0, 0, 0);
    F0[0] = (M(2, 0) + M(3, 0)) / 2.0;
    F0[1] = (M(4, 1) + M(5, 1)) / 2.0;
    F0[2] = (M(0, 2) + M(1, 2)) / 2.0;

    // 计算力矩

    auto T0 =std::vector<double>{0, 0, 0};
    T0[0] = (M(0, 3) + M(1, 3) + M(4, 3) + M(5, 3)) / 4.0;
    T0[1] = (M(0, 4) + M(1, 4) + M(2, 4) + M(3, 4)) / 4.0;
    T0[2] = (M(2, 5) + M(3, 5) + M(4, 5) + M(5, 5)) / 4.0;
    double G;
    G = (M(1, 2) - M(0, 2) + M(3, 0) - M(2, 0) + M(5, 1) - M(4, 1)) / 6.0;

    // 计算质心

    std::vector<double> L{0, 0, 0};
    L[0] = (M(0, 4) - M(1, 4) + M(5, 5) - M(4, 5)) / 4.0 / G;
    L[1] = (M(1, 3) - M(0, 3) + M(2, 5) - M(3, 5)) / 4.0 / G;
    L[2] = (M(3, 4) - M(2, 4) + M(4, 3) - M(5, 3)) / 4.0 / G;

    std::vector<double> result{F0[0], F0[1], F0[2], T0[0], T0[1], T0[2], G, L[0], L[1], L[2]};
    return result;
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
    double joints_start[7] = {0, 0, 0, PI / 2, 0, -PI / 2, 0}; // 以 7 轴机器人为例

    // double joints[JOINT_NUM] = {0.0};
    double poses[6] = {0.0};
    // 初始点位
    moveJToTarget(joints_start, 0.5, 0.5, 0, 0, 0, strIpAddress);
    wait_move(strIpAddress);
    ret = getTcpPos(poses, strIpAddress);
    
    if (ret < 0)
    {
        printf("getJointPos failed! Return value = %d\n", ret);
    }
    else
    {
        // std::cout << "joints: " << joints[0] << " " << joints[1] << " " << joints[2] << " " << joints[3] << " " << joints[4] << " " << joints[5] << " " << joints[6] << std::endl;
    }

    
    //质量与质心
    double mass = 6.22721;
    KDL::Vector center_of_mass_position(0.0, 0.0, 0.0366358);
    
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
    KDL::Vector rr = frame.M.GetRot();
    // KDL::Wrench wrench;
    double pose_target[6] = {frame.p.x(), frame.p.y(), frame.p.z(), rr.x(), rr.y(), rr.z()};


    
    getTcpPos(poses, strIpAddress);
    

    
    KDL::Rotation f_TCP = KDL::Rotation::RPY(M_PI,0,-M_PI/4);
    
    // enable_wrench[0] = enable_wrench[0] +0.59148;
    // enable_wrench[1] = enable_wrench[1] +4.38301;
    // enable_wrench[2] = enable_wrench[2] +9.24931;
    
        // KDL::Vector sensor_base= f_base*KDL::Vector(enable_wrench[0], enable_wrench[1], enable_wrench[2]);
        // std::cout<<"sensor_base: "<<sensor_base.x()<<","<<sensor_base.y()<<","<<sensor_base.z()<<std::endl;


    KDL::Rotation r1 = KDL::Rotation::RPY(0, 0, 0)*f_TCP.Inverse();
    KDL::Rotation r2 = KDL::Rotation::RPY(M_PI, 0, M_PI)*f_TCP.Inverse();
    KDL::Rotation r3 = KDL::Rotation::RPY(0, -M_PI/2, 0)*f_TCP.Inverse() ;
    KDL::Rotation r4 = KDL::Rotation::RPY(0, M_PI/2, 0)*f_TCP.Inverse();
    KDL::Rotation r6 = KDL::Rotation::RPY(-M_PI/2, 0, -M_PI/2)*f_TCP.Inverse();
    KDL::Rotation r5 = KDL::Rotation::RPY(M_PI/2, 0, M_PI/2)*f_TCP.Inverse();
    
    std::vector<std::array<double,6>> pose_mass;
    std::vector<std::array<double,7>> six_joints;
    std::vector<SixDForce> sixDForce;
    sixDForce.resize(6);
    pose_mass.push_back({poses[0], poses[1], poses[2], r1.GetRot().x(), r1.GetRot().y(), r1.GetRot().z()});
    pose_mass.push_back({poses[0], poses[1], poses[2], r2.GetRot().x(), r2.GetRot().y(), r2.GetRot().z()});
    pose_mass.push_back({poses[0], poses[1], poses[2], r3.GetRot().x(), r3.GetRot().y(), r3.GetRot().z()});
    pose_mass.push_back({poses[0], poses[1], poses[2], r4.GetRot().x(), r4.GetRot().y(), r4.GetRot().z()});
    pose_mass.push_back({poses[0], poses[1], poses[2], r5.GetRot().x(), r5.GetRot().y(), r5.GetRot().z()});
    pose_mass.push_back({poses[0], poses[1], poses[2], r6.GetRot().x(), r6.GetRot().y(), r6.GetRot().z()});
    double six_poses0[6] = {0.0123974,-0.575365,0.69058,-2.90244,1.20223,-1.09227e-05};
    double six_poses1[6] = {0.163479,-0.583128,0.665763,2.28035e-05,1.18628e-05,-2.35617};
    double six_poses2[6] = {-0.0496656,-0.598473,0.764768,-1.75998,0.729006,-1.76002};
    double six_poses3[6] = {0.0474273,-0.595513,0.766014,1.76,-0.729018,-1.75999};
    double six_poses4[6] = {0.191334,-0.557764,0.718021,0.613939,-1.4822,-0.61397};
    double six_poses5[6] = {-0.0102881,-0.622588,0.798573,-1.76001,-0.729027,1.75996};

    six_joints.push_back({-0.114833,-0.672974,0.144589,2.21393,-3.05164,-1.54613,0.784655});
    six_joints.push_back({-1.45441,0.436106,1.7513,1.73178,-0.429719,-1.50033,-2.00883});
    six_joints.push_back({-0.923565,-0.324006,1.82484,1.55034,1.83609,-2.24518,-0.863044});
    six_joints.push_back({-1.59715,-0.0464151,3.11139,1.68565,-1.57849,-1.51477,-0.623923});
    six_joints.push_back({-1.59719,-0.0464151,3.11139,1.68563,-1.57851,-1.51476,-2.19472});
    six_joints.push_back({-1.59841,-0.053354,3.10717,1.67615,-1.57879,-1.50929,0.944405});
 
    int sum = 1000;
    double vel = 0.1;
    double acc = 0.1;
    double radius = 0.0;
    int zv_shaper_order = 0;
    double zv_shaper_frequency = 0;
    double zv_shaper_damping_ratio = 0;

    // moveJToTarget(six_joints2, 0.2, 0.2, 0, 0, 0, strIpAddress);
    // wait_move(strIpAddress);
    // sleep(3);
    

    // KDL::Rotation target_r= KDL::Rotation::RPY(M_PI/2, 0, 0)*f_TCP.Inverse();
    // double pose_start[6] = {0.13,-0.563973,0.721639,0.613945,-1.48222,-0.613962};
    // ret = moveLToPose(pose_start, vel, acc, nullptr, zv_shaper_order, zv_shaper_frequency, zv_shaper_damping_ratio, strIpAddress);
    // wait_move(strIpAddress);
    double B = 20000.0;
    for(int i=0;i<6;i++)
    {
        sleep(1);
        moveJToTarget(six_joints[i].data(), 0.5, 0.5, 0, 0, 0, strIpAddress);
        // ret = moveLToPose(pose_mass[i].data(), vel, acc, nullptr, zv_shaper_order, zv_shaper_frequency, zv_shaper_damping_ratio, strIpAddress);
        wait_move(strIpAddress);

        for (int k = 0; k < sum; k++)
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

        // sixDForce.push_back({enable_wrench[0],enable_wrench[1],enable_wrench[2],enable_wrench[3],enable_wrench[4],enable_wrench[5]});
        sixDForce[i].force_x = enable_wrench[0];
        sixDForce[i].force_y = enable_wrench[1];
        sixDForce[i].force_z = enable_wrench[2];
        sixDForce[i].torque_roll = enable_wrench[3];
        sixDForce[i].torque_pitch = enable_wrench[4];
        sixDForce[i].torque_yaw = enable_wrench[5];
        std::cout << "sixDForce[i]: " << sixDForce[i].force_x << "," << sixDForce[i].force_y << "," << sixDForce[i].force_z << "," << sixDForce[i].torque_roll << "," << sixDForce[i].torque_pitch << "," << sixDForce[i].torque_yaw  << std::endl;

    }
    
   
    // ret = moveLToPose(pose_mass[0].data(), vel, acc, nullptr, zv_shaper_order, zv_shaper_frequency, zv_shaper_damping_ratio, strIpAddress);
    // wait_move(strIpAddress);
    // double pose_end[6] = {poses[0], poses[1], poses[2], target_r.GetRot().x(), target_r.GetRot().y(), target_r.GetRot().z()};
    // sleep(5);
    // ret = moveLToPose(pose_end, vel, acc, nullptr, zv_shaper_order, zv_shaper_frequency, zv_shaper_damping_ratio, strIpAddress);
    // wait_move(strIpAddress);

    // 上电采集力信息数据

    
    
    getTcpPos(poses, strIpAddress);
    
    SixDForceTool sixDForceTool;
    KDL::Wrench realtime_wrench;
    std::cout << "enable_wrench: " << enable_wrench[0] << "," << enable_wrench[1] << "," << enable_wrench[2] << "," << enable_wrench[3] << "," << enable_wrench[4] << "," << enable_wrench[5] << std::endl;
    std::cout << "tcp pos: " << poses[0] << "," << poses[1] << "," << poses[2]<< "," << poses[3] << "," << poses[4] << "," << poses[5]<< std::endl;

    // realtime_wrench.force((enable_wrench[0],enable_wrench[1],enable_wrench[2]));
    // realtime_wrench.torque((enable_wrench[3],enable_wrench[4],enable_wrench[5]));
    
    // getTcpPos(poses, strIpAddress);
    
    // KDL::Vector axis = KDL::Vector(poses[3], poses[4], poses[5]);
    // double norm = axis.Normalize();
    // KDL::Frame TCP_base = KDL::Frame(KDL::Rotation::Rot(axis, norm), KDL::Vector(poses[0], poses[1], poses[2]));
    // KDL::Rotation f_TCP = KDL::Rotation::RPY(M_PI,0,-M_PI/4);
    // KDL::Rotation f_base = f_TCP * TCP_base.M ;

    
    // SixDForce sixDForce[6];
    // sixDForce[0] = { -4.04905,-5.81165,-23.0022,0.188509,-0.803967,-0.0391097};
    // sixDForce[1] = { -3.23488,-4.88059,7.19731,-0.0434611,0.20479,-0.0529627};
    // sixDForce[2] = { -18.9389,-5.92461,-8.28863,0.143034,-1.14555,-0.0931772};
    // sixDForce[3] = { 11.403,-5.58718,-8.03096,0.115094,0.463372,0.00958173};
    // sixDForce[5] = { -3.68731,8.33788,-8.15823,-0.666457,-0.347675,-0.486904};
    // sixDForce[4] = { -3.82365,-21.1465,-7.96194,0.937887,-0.322435,0.412095};

    sixDForceTool.forces = sixDForce;
    // for(int i=0;i<6;i++)
    // {
    //   sixDForceTool.addData(sixDForce[i]);
    // }
    
    // sixDForceTool.LoadParameterIdentification(realtime_wrench,f_base);
    sixDForceTool.GetMassAndGravity();
    double my_mass=sixDForceTool.m_mass;
    double my_massx=sixDForceTool.m_massx;
    double my_massy=sixDForceTool.m_massy;
    double my_massz=sixDForceTool.m_massz;
    sixDForceTool.GetZeroOffset();
    std::cout << "my_mass: " << my_mass << std::endl;
    std::cout << "my_massx: " << my_massx << std::endl;
    std::cout << "my_massy: " << my_massy << std::endl;
    std::cout << "my_massz: " << my_massz << std::endl;
    

    moveJToTarget(joints_start, 0.5, 0.5, 0, 0, 0, strIpAddress);
    wait_move(strIpAddress);
    sleep(3);
    ret = moveLToPose(pose_target, vel, acc, nullptr, zv_shaper_order, zv_shaper_frequency, zv_shaper_damping_ratio, strIpAddress);
    wait_move(strIpAddress);
    sensor.startRealTimeDataRepeatedly<float>(&rtDataHandler, rtMode, rtDataValid);
    getTcpPos(poses, strIpAddress);
    
    while(isRunning)
    {
    //    isRunning = false;
        
            
        // auto rtData = sensor.getRealTimeDataOnce<float>(rtMode, rtDataValid);

        // for (int i = 0; i < rtData.size(); i++)
        // {
        //     for (int j = 0; j < 6; j++)
        //     {
        //     // std::cout << "Ch " << j << ": " << rtData[i][j] << "\t";
        //         enable_wrench[j] = rtData[i][j];
        //     }
        // }
            
            
            
        if (getRobotState(strIpAddress) == 6)
        {
            noError = false;
        }

        if(noError)
        {
            
            KDL::Vector axis = KDL::Vector(poses[3], poses[4], poses[5]);
            double norm = axis.Normalize();
            KDL::Frame TCP_base = KDL::Frame(KDL::Rotation::Rot(axis, norm), KDL::Vector(poses[0], poses[1], poses[2]));
            KDL::Rotation f_base = TCP_base.M*f_TCP;

            

            realtime_wrench.force.data[0] = enable_wrench[0];
            realtime_wrench.force.data[1] = enable_wrench[1];
            realtime_wrench.force.data[2] = enable_wrench[2];
            realtime_wrench.torque.data[0] = enable_wrench[3];
            realtime_wrench.torque.data[1] = enable_wrench[4];
            realtime_wrench.torque.data[2] = enable_wrench[5];
            // std::cout << "realtime_wrench: " << realtime_wrench.force.data[0]<<","<<realtime_wrench.force.data[1]<<","<< realtime_wrench.force.data[2]<<","<<realtime_wrench.torque.data[0]<<","<<realtime_wrench.torque.data[1]<<","<<realtime_wrench.torque.data[2]<< std::endl;
            
            KDL::Wrench wrench_compensation = sixDForceTool.GravityCompensation(realtime_wrench,f_base);;
            std::cout << "wrench_compensation:" << wrench_compensation.force.data[0] <<","<< wrench_compensation.force.data[1]<<","<< wrench_compensation.force.data[2]<<","<<wrench_compensation.torque.data[0]<<","<<wrench_compensation.torque.data[1]<<","<<wrench_compensation.torque.data[2]<< std::endl;
            wrench_compensation.force.x(filter_array[0].filter(wrench_compensation.force.x()));
            wrench_compensation.force.y(filter_array[1].filter(wrench_compensation.force.y()));
            wrench_compensation.force.z(filter_array[2].filter(wrench_compensation.force.z()));

            //死区
            if (abs(wrench_compensation.force.x()) < 1)
            {
                wrench_compensation.force.x(0);
            }
            if (abs(wrench_compensation.force.y()) < 1)
            {
                wrench_compensation.force.y(0);
            }

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
            KDL::Wrench wrench_base = TCP_base.M * f_TCP * wrench_compensation;

            poses[0] += wrench_base.force.x() / B;
            poses[1] += wrench_base.force.y() / B;
            poses[2] += wrench_base.force.z() / B;

            axis = KDL::Vector(poses[3], poses[4], poses[5]);
            auto n = normal;
            n.Normalize();
            double my_norm = axis.Normalize();
            double angle = -wrench_compensation.torque.z() / 1000.0;
            auto delta_R = KDL::Rotation::Rot(n, angle);
            auto m = delta_R * KDL::Rotation::Rot(axis, my_norm);
            TCP_base = KDL::Frame(m, KDL::Vector(poses[0], poses[1], poses[2]));
            KDL::Vector r_pose = TCP_base.M.GetRot();
            poses[3] = r_pose.x();
            poses[4] = r_pose.y();
            poses[5] = r_pose.z();

            axis = KDL::Vector(poses[3], poses[4], poses[5]);
            auto my_n = normal;
            my_n.Normalize();
            my_norm = axis.Normalize();
            double my_angle = -wrench_compensation.torque.z() / 1000.0;
            delta_R = KDL::Rotation::Rot(my_n, my_angle);
            m = delta_R * KDL::Rotation::Rot(axis, my_norm);
            TCP_base = KDL::Frame(m, KDL::Vector(poses[0], poses[1], poses[2]));
            KDL::Vector r = TCP_base.M.GetRot();
            poses[3] = r.x();
            poses[4] = r.y();
            poses[5] = r.z();

            servoL(poses, 0.01, 0.1, 300, 1.0, nullptr, strIpAddress);
            usleep(1000);
        }
        else
        {
            noError = true;
            getTcpPos(poses, strIpAddress);
            cleanErrorInfo(strIpAddress);
            setLastError(0, strIpAddress);           
            usleep(1000);
        }
    }
    


    destroySrv(strIpAddress);
    return 0;
}
