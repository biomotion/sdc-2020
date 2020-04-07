#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"
#include<Eigen/Dense>
#include<Eigen/Geometry>
#include<cmath>
#include<iostream>

class EkfFusion{
private:
    //ROS essentials
    ros::NodeHandle n;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_odom_combined;
    ros::Subscriber sub_zed_odom;
    ros::Publisher pub_marker;
    ros::Publisher pub_imu_transformed;
    visualization_msgs::Marker imu_marker, combined_marker, zed_marker;

    Eigen::Matrix3d ItoZO;
    //
    //R body to global (3x3)
    Eigen::Matrix3d C;
    Eigen::Vector3d gravity;
    Eigen::Vector3d line_vel, pos;
    ros::Time last_enter;
    void update_imu_orientation(const geometry_msgs::Vector3& ang_vel, const ros::Duration time_diff){
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
        // std::cout << sigma << std::endl;
        C = C * (Eigen::Matrix3d::Identity() + trans*(sin(sigma)/sigma) + trans*trans*(((double)1-cos(sigma))/(sigma*sigma)) );
        // std::cout << C << std::endl;

    }
    void update_imu_position(const geometry_msgs::Vector3& lin_accel, const ros::Duration time_diff){
        Eigen::Vector3d glo_acc = C * Eigen::Vector3d(lin_accel.x, lin_accel.y, lin_accel.z);
        // std::cout << glo_acc.transpose() << std::endl;
        this->line_vel += (glo_acc - gravity)*(double)time_diff.toNSec()/1e9;
        this->pos += this->line_vel*(double)time_diff.toNSec()/1e9;
    }

    void transform_imu(const sensor_msgs::Imu::ConstPtr& msg){
        // Imu transform to zed frame
        sensor_msgs::Imu imu_tf_msg;
        imu_tf_msg.header = msg->header;
        imu_tf_msg.header.frame_id = "map";
        Eigen::Vector3d ang_vel = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z), ang_vel_tf;
        Eigen::Vector3d lin_accel = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z), lin_accel_tf;
        Eigen::Matrix3d orien(Eigen::Quaterniond(msg->orientation.w,
                                                msg->orientation.x,
                                                msg->orientation.y,
                                                msg->orientation.z));
        Eigen::Quaterniond orien_tf(ItoZO * orien);
        ang_vel_tf = ItoZO * ang_vel;
        lin_accel_tf = ItoZO * lin_accel;

        imu_tf_msg.orientation.w = orien_tf.w();
        imu_tf_msg.orientation.x = orien_tf.x();
        imu_tf_msg.orientation.y = orien_tf.y();
        imu_tf_msg.orientation.z = orien_tf.z();
        imu_tf_msg.angular_velocity.x = ang_vel_tf(0);
        imu_tf_msg.angular_velocity.y = ang_vel_tf(1);
        imu_tf_msg.angular_velocity.z = ang_vel_tf(2);
        imu_tf_msg.linear_acceleration.x = lin_accel_tf(0);
        imu_tf_msg.linear_acceleration.y = lin_accel_tf(1);
        imu_tf_msg.linear_acceleration.z = lin_accel_tf(2);

        this->pub_imu_transformed.publish(imu_tf_msg);
    }

    void imu_msg_cb(const sensor_msgs::Imu::ConstPtr& msg){
        // ROS_INFO("cb");
        this->transform_imu(msg);

        // Imu Odometry implement below
        ros::Time enter_time = msg->header.stamp;
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
        this->update_imu_orientation(msg->angular_velocity, enter_time-this->last_enter);
        this->update_imu_position(msg->linear_acceleration, enter_time-this->last_enter);
        
        //draw imu marker
        geometry_msgs::Point point;
        Eigen::Vector3d pos_tf = ItoZO * pos;
        point.x = pos_tf(0); point.y = pos_tf(1); point.z = pos_tf(2);
        imu_marker.id++;
        imu_marker.action = visualization_msgs::Marker::ADD;
        imu_marker.points.push_back(point);
        imu_marker.header.stamp = enter_time;
        this->pub_marker.publish( imu_marker );

        this->last_enter = enter_time;
        return;
    }

    void odom_combined_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
        geometry_msgs::Point point;
        point.x = msg->pose.pose.position.x;
        point.y = msg->pose.pose.position.y;
        point.z = msg->pose.pose.position.z;
        combined_marker.id++;
        combined_marker.action = visualization_msgs::Marker::ADD;
        combined_marker.points.push_back(point);
        combined_marker.header.stamp = msg->header.stamp;
        this->pub_marker.publish( combined_marker );

        return;
    }

    void zed_odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
        geometry_msgs::Point point;
        point.x = msg->pose.pose.position.x;
        point.y = msg->pose.pose.position.y;
        point.z = msg->pose.pose.position.z;
        zed_marker.id++;
        zed_marker.action = visualization_msgs::Marker::ADD;
        zed_marker.points.push_back(point);
        zed_marker.header.stamp = msg->header.stamp;
        this->pub_marker.publish( zed_marker );
        return;
    }
public:
    EkfFusion() {}
    EkfFusion(ros::NodeHandle nh){
        this->n = nh;
        ROS_INFO("Node Initializing");
        this->last_enter = ros::Time(0);
        this->sub_imu = n.subscribe("/imu/data", 1, &EkfFusion::imu_msg_cb, this);
        this->sub_odom_combined = n.subscribe("/robot_pose_ekf/odom_combined", 1, &EkfFusion::odom_combined_cb, this);
        this->sub_zed_odom = n.subscribe("/zed/odom", 1, &EkfFusion::zed_odom_cb, this);
        this->pub_marker = n.advertise<visualization_msgs::Marker>("/position", 1);
        this->pub_imu_transformed = n.advertise<sensor_msgs::Imu>("/imu_data", 1);

        imu_marker.type = visualization_msgs::Marker::LINE_STRIP;
        imu_marker.id = 0;
        imu_marker.header.frame_id = "map";
        imu_marker.ns = "imu_pose";
        imu_marker.scale.x = 0.1;
        imu_marker.color.a = 1.0;
        imu_marker.color.r = 0.0;
        imu_marker.color.g = 0.0;
        imu_marker.color.b = 1.0;

        combined_marker.type = visualization_msgs::Marker::LINE_STRIP;
        combined_marker.id = 0;
        combined_marker.header.frame_id = "map";
        combined_marker.ns = "odom_combined";
        combined_marker.scale.x = 0.1;
        combined_marker.color.a = 1.0;
        combined_marker.color.r = 0.0;
        combined_marker.color.g = 1.0;
        combined_marker.color.b = 0.0;

        zed_marker.type = visualization_msgs::Marker::LINE_STRIP;
        zed_marker.id = 0;
        zed_marker.header.frame_id = "map";
        zed_marker.ns = "zed_odom";
        zed_marker.scale.x = 0.1;
        zed_marker.color.a = 1.0;
        zed_marker.color.r = 1.0;
        zed_marker.color.g = 0.0;
        zed_marker.color.b = 0.0;
        

        Eigen::Matrix3d ItoCam, CamtoZO;
        ItoCam <<   0.0225226,  0.999745,       0.0017194,
                    0.0648765,  -0.00317777,    0.997888,
                    0.997639,   -0.0223635,     -0.0649315;
        CamtoZO <<  0,  0, 1,
                    -1, 0, 0,
                    0, -1, 0;
        std::cout << "ItoCam: " << std::endl << ItoCam << std::endl;
        std::cout << "CamtoZO: " << std::endl << CamtoZO << std::endl;
        
        ItoZO = CamtoZO*ItoCam;
        std::cout << "ItoZO: " << std::endl << ItoZO << std::endl;
        ROS_INFO("Initialize Done");
    }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "ekf_fusion");
    ros::NodeHandle n;
    EkfFusion node(n);
    ros::spin();
    return 0;
}