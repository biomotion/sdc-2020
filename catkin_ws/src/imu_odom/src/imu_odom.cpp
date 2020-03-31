#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include<Eigen/Dense>
#include<cmath>
#include<iostream>

class ImuOdom{
private:
    //ROS essentials
    ros::NodeHandle n;
    ros::Subscriber sub_imu;
    ros::Publisher pub_marker;
    uint16_t marker_id;
    //
    //R body to global (3x3)
    Eigen::Matrix3d C;
    Eigen::Vector3d gravity;
    Eigen::Vector3d line_vel, pos;
    ros::Time last_enter;
    void update_orientation(const geometry_msgs::Vector3& ang_vel, const ros::Duration time_diff){
        Eigen::Matrix3d trans = Eigen::Matrix3d::Zero();
        double sigma = (Eigen::Vector3d(ang_vel.x, ang_vel.y, ang_vel.z)*(double)time_diff.toNSec()/1e9).norm();
        // Eigen::Vector3d sigma(ang_vel);
        // sigma *= (double)time_diff.toNSec()/1e9;
        // ang_vel x duration
        trans(0, 1) = -ang_vel.z;
        trans(0, 2) = ang_vel.y;
        trans(1, 0) = ang_vel.z;
        trans(1, 2) = -ang_vel.x;
        trans(2, 0) = -ang_vel.y;
        trans(2, 1) = ang_vel.x;
        trans *= (double)time_diff.toNSec()/1e9;
        // std::cout << trans << std::endl;
        
        C = C * (Eigen::Matrix3d::Identity() + trans*(sin(sigma)/sigma) + trans*trans*(((double)1-cos(sigma)/sigma*sigma)) );
        // std::cout << C << std::endl;

    }
    void update_position(const geometry_msgs::Vector3& lin_accel, const ros::Duration time_diff){
        Eigen::Vector3d glo_acc = C * Eigen::Vector3d(lin_accel.x, lin_accel.y, lin_accel.z);
        std::cout << glo_acc.transpose() << std::endl;
        this->line_vel += (glo_acc - gravity)*(double)time_diff.toNSec()/1e9;
        this->pos += this->line_vel*(double)time_diff.toNSec()/1e9;


    }

    void msg_cb(const sensor_msgs::Imu::ConstPtr& msg){
        // ROS_INFO("cb");
        // Implement here
        ros::Time enter_time = ros::Time::now();
        // std::cout << "ang_vel= \n" << msg->angular_velocity;
        // std::cout << "time_diff = " << enter_time - this->last_enter << std::endl;
        if(last_enter == ros::Time(0)){
            ROS_INFO("First msg");
            this->C = Eigen::Matrix3d::Identity(); // initial orientation
            this->line_vel = Eigen::Vector3d::Zero();
            this->pos = Eigen::Vector3d::Zero();
            this->gravity = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);  //Initialize gravity
            this->last_enter = enter_time;
            return;
        }
        //Propergate
        this->update_orientation(msg->angular_velocity, enter_time-this->last_enter);
        this->update_position(msg->linear_acceleration, enter_time-this->last_enter);
        
        //draw marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "global";
        marker.header.stamp = ros::Time::now();
        marker.ns = "imu_pose";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pos(0);
        marker.pose.position.y = pos(1);
        marker.pose.position.z = pos(2);
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        this->pub_marker.publish( marker );

        this->last_enter = enter_time;
        return;
    }
public:
    ImuOdom() {}
    ImuOdom(ros::NodeHandle nh){
        this->n = nh;
        ROS_INFO("Node Initializing");
        this->last_enter = ros::Time(0);
        this->sub_imu = n.subscribe("/imu/data", 1, &ImuOdom::msg_cb, this);
        this->pub_marker = n.advertise<visualization_msgs::Marker>("/position", 1);
        marker_id = 0;
        ROS_INFO("Initialize Done");
    }



};

int main(int argc, char** argv){
    ros::init(argc, argv, "imu_odom");
    ros::NodeHandle n;
    ImuOdom node(n);
    ros::spin();
    return 0;
}