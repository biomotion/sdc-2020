#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include<Eigen/Dense>
#include<iostream>

class ImuOdom{
private:
    //ROS essentials
    ros::NodeHandle n;
    ros::Subscriber sub_imu;

    //
    //R body to global (3x3)
    Eigen::Matrix3d C;

    void msg_cb(const sensor_msgs::Imu::ConstPtr& msg){
        ROS_INFO("cb");
        // Implement here
    }
public:
    ImuOdom() {}
    ImuOdom(ros::NodeHandle nh){
        this->n = nh;
        ROS_INFO("Node initializing");
        this->sub_imu = n.subscribe("/imu/data", 1, &ImuOdom::msg_cb, this);
        this->C = Eigen::Matrix3d::Identity();
        // std::cout << C << std::endl;
    }



};

int main(int argc, char** argv){
    ros::init(argc, argv, "imu_odom");
    ros::NodeHandle n;
    ImuOdom node(n);
    ros::spin();
    return 0;
}